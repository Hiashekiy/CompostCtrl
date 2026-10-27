"""Unified MPC for the aero-surface + CMG actuator set.

Decision variables: u = [delta_c (3), gamma_dot_c (4)] -- the controller directly
commands the three aerodynamic surfaces and the four CMG gimbal rates. Because
the gimbal rates are optimisation variables, the MPC *is* the steering law
(no separate pseudo-inverse allocation).

Cost terms
    attitude error (exact vector part of the error quaternion, linear in q)
    body-rate error
    surface deflection effort
    gimbal-rate effort
    gimbal-angle deviation from the home configuration
    CMG cluster momentum  ||H_cmg(gamma)||^2   (soft desaturation; the aero
        surfaces provide the external torque that actually unloads it)
Constraints
    dynamics, surface position/rate reachability,
    gimbal rate / rate-reachability limits, singularity floor on sigma_min(A)
    (linearised, with a slack penalty).
"""
from __future__ import annotations

import time

import numpy as np

from nearspace.cmg import CMGCluster
from nearspace.model6 import cross_matrix, quat_conj, quat_mul
from nearspace.model6c import (NX, NU, VehicleCMGParams, discretize6c, dynamics6c,
                      limit_commands6c, plant_step6c)
from legacy.mpc import QPSolver


class LTVMPCCMG:
    def __init__(self, prm: VehicleCMGParams, cfg, cluster: CMGCluster | None = None):
        self.prm = prm
        self.cfg = cfg
        self.cluster = cluster if cluster is not None else CMGCluster(prm.cmg)
        m = getattr(cfg, "mpc_cmg", None) or cfg.mpc3
        self.dts = np.concatenate([np.full(int(seg.n), float(seg.dt)) for seg in m.horizon])
        self.N = int(len(self.dts))
        self.Q_att = float(m.q_att) * np.eye(3)
        self.Q_rate = float(m.q_rate) * np.eye(3)
        self.R_delta = float(m.r_delta) * np.eye(3)
        self.R_gdot = float(getattr(m, "r_gdot", 1.0e-4)) * np.eye(4)
        self.W_gamma = float(getattr(m, "w_gamma", 1.0))
        self.W_H = float(getattr(m, "w_H", 5.0e-4))
        self.s_delta = float(m.s_delta)
        self.rho_sing = float(getattr(m, "rho_sing", 1.0e3))
        self.term = float(m.terminal_scale)
        self.n_sqp = int(m.n_sqp)
        self.qp = QPSolver(max_iter=int(getattr(m, "qp_max_iter", 20000)),
                           eps_abs=1e-5, eps_rel=1e-5, internal_scaling=False)
        self.k_att = float(getattr(m, "k_att", 4.0))
        self.k_rate = float(getattr(m, "k_rate", 4.0))
        self.k_unload = float(getattr(m, "k_unload", 0.05))
        self.sigma_floor = prm.cmg.sigma_floor
        self.u_prev = np.zeros(NU)
        self.x_seq = None
        self.u_seq = None
        self.z_warm = None
        self.last_solve_time = 0.0
        self.last_viol = 0.0
        self.att_err = 0.0
        self.min_sigma = 0.0

    # ---- indices --------------------------------------------------------
    def _xi(self, k):
        return NX * (k - 1)

    def _ui(self, k):
        return NX * self.N + NU * k

    def _si(self, k):
        return NX * self.N + NU * self.N + k

    # ---- attitude error -------------------------------------------------
    @staticmethod
    def _err_map(q_ref):
        w_r = q_ref[0]
        v_r = q_ref[1:4]
        E = np.zeros((3, 4))
        E[:, 0] = -2.0 * v_r
        E[:, 1:4] = 2.0 * (w_r * np.eye(3) - cross_matrix(v_r))
        return E

    @staticmethod
    def _attitude_error(q_ref, q):
        dq = quat_mul(quat_conj(q_ref), q)
        if dq[0] < 0.0:
            dq = -dq
        return 2.0 * dq[1:4]

    # ---- initial guess / failsafe --------------------------------------
    def _pd_sequence(self, x0, traj, t_seq, u_guess=None):
        v, c = self.prm.veh, self.prm.cmg
        us = np.zeros((self.N, NU)) if u_guess is None else np.array(u_guess, float)
        A = self.cluster.jacobian(x0[10:14])
        for k in range(self.N):
            t = t_seq[k]
            e = self._attitude_error(traj.q_ref(t), x0[0:4])
            w_err = x0[4:7] - traj.omega_ref(t)
            M_des = v.J if False else np.diag([v.Jx, v.Jy, v.Jz]) @ (self.k_att * e + self.k_rate * w_err)
            # damped pseudo-inverse steering for the CMG
            lam = 0.05 * c.h0
            gdot = -A.T @ np.linalg.solve(A @ A.T + lam ** 2 * np.eye(3), M_des)
            gdot = np.clip(gdot, -c.gdot_max, c.gdot_max)
            M_cmg = A @ gdot
            M_res = M_des - M_cmg
            Q = 0.5 * traj.rho(t) * traj.V(t) ** 2
            alpha_ref = traj.pitch_ref(t) - traj.gamma(t)
            Ma_cap = Q * v.S * np.array([v.b * v.Cl_da, v.cbar * v.Cm_de, v.b * v.Cn_dr])
            M_rest = np.array([0.0, Q * v.S * v.cbar * (v.Cm0 + v.Cm_alpha * alpha_ref), 0.0])
            denom = np.where(np.abs(Ma_cap) > 1e-9, Ma_cap, 1e-9)
            d_moment = (M_res - M_rest) / denom
            d_state = np.clip(np.array([d_moment[1], d_moment[0], d_moment[2]]),
                              -v.delta_max, v.delta_max)
            us[k, 0:3] = d_state
            us[k, 3:7] = gdot
        return us

    def _initial_rollout(self, x0, p_seq, u_seq):
        xs = [np.array(x0, float)]
        x = np.array(x0, float)
        for u, p, dt in zip(u_seq, p_seq, self.dts):
            n = max(1, int(np.ceil(dt / 0.01)))
            for _ in range(n):
                x, _ = plant_step6c(x, u, p, dt / n, self.prm, self.cluster, substeps=1)
            xs.append(x.copy())
        return np.array(xs)

    # ---- QP -------------------------------------------------------------
    def _build_qp(self, x0, x_seq, u_seq, p_seq, t_seq, traj):
        N = self.N
        v, c = self.prm.veh, self.prm.cmg
        nz = (NX + NU + 1) * N
        H = np.zeros((nz, nz))
        g = np.zeros(nz)
        rows, lbs, ubs = [], [], []

        def add(coeffs, lb, ub):
            rows.append(coeffs)
            lbs.append(lb)
            ubs.append(ub)

        # ---- cost ----
        for k in range(1, N + 1):
            dt = self.dts[k - 1]
            t = t_seq[k]
            qr = traj.q_ref(t)
            E = self._err_map(qr)
            He = E.T @ self.Q_att @ E
            qi = self._xi(k)
            for a in range(4):
                for b in range(4):
                    H[qi + a, qi + b] += 2.0 * He[a, b] * dt
            w_ref = traj.omega_ref(t)
            for a in range(3):
                H[qi + 4 + a, qi + 4 + a] += 2.0 * self.Q_rate[a, a] * dt
                g[qi + 4 + a] += -2.0 * self.Q_rate[a, a] * dt * w_ref[a]
            # gimbal angle preference (home configuration)
            for a in range(4):
                H[qi + 10 + a, qi + 10 + a] += 2.0 * self.W_gamma * dt
            # CMG momentum penalty: H_cmg ~ H_ref + A_ref (gamma - gamma_ref)
            Href = self.cluster.momentum(x_seq[k][10:14])
            Aref = self.cluster.jacobian(x_seq[k][10:14])
            for a in range(4):
                for b in range(4):
                    H[qi + 10 + a, qi + 10 + b] += 2.0 * self.W_H * dt * (Aref[:, a] @ Aref[:, b])
                g[qi + 10 + a] += 2.0 * self.W_H * dt * (Aref[:, a] @ (Href - Aref @ x_seq[k][10:14]))
        for k in range(N):
            dt = self.dts[k]
            ui = self._ui(k)
            for a in range(3):
                H[ui + a, ui + a] += 2.0 * self.R_delta[a, a] * dt
            for a in range(4):
                H[ui + 3 + a, ui + 3 + a] += 2.0 * self.R_gdot[a, a] * dt
        for k in range(1, N):
            H[self._ui(k), self._ui(k)] += 2.0 * self.s_delta
            H[self._ui(k), self._ui(k - 1)] += -2.0 * self.s_delta
            H[self._ui(k - 1), self._ui(k - 1)] += 2.0 * self.s_delta
        H[self._ui(0), self._ui(0)] += 2.0 * self.s_delta
        g[self._ui(0)] += -2.0 * self.s_delta * self.u_prev[0]
        for k in range(N):
            H[self._si(k), self._si(k)] += 2.0 * self.rho_sing
        # terminal
        tN = t_seq[N]
        qr = traj.q_ref(tN)
        E = self._err_map(qr)
        He = self.term * (E.T @ self.Q_att @ E)
        Pi = self._xi(N)
        for a in range(4):
            for b in range(4):
                H[Pi + a, Pi + b] += 2.0 * He[a, b]
        w_ref = traj.omega_ref(tN)
        for a in range(3):
            H[Pi + 4 + a, Pi + 4 + a] += 2.0 * self.term * self.Q_rate[a, a]
            g[Pi + 4 + a] += -2.0 * self.term * self.Q_rate[a, a] * w_ref[a]

        dmax, ddot = v.delta_max, v.delta_dot_max
        gdmax, gddmax = c.gdot_max, c.gddot_max

        # ---- dynamics ----
        for k in range(N):
            Ad, Bd, d0 = discretize6c(x_seq[k], u_seq[k], p_seq[k], self.prm, self.cluster, self.dts[k])
            for i in range(NX):
                coeff = {self._xi(k + 1) + i: 1.0}
                if k == 0:
                    rhs = d0[i] + (Ad @ x0)[i]
                else:
                    for j in range(NX):
                        if Ad[i, j] != 0.0:
                            coeff[self._xi(k) + j] = coeff.get(self._xi(k) + j, 0.0) - Ad[i, j]
                    rhs = d0[i]
                for j in range(NU):
                    if Bd[i, j] != 0.0:
                        coeff[self._ui(k) + j] = coeff.get(self._ui(k) + j, 0.0) - Bd[i, j]
                add(coeff, rhs, rhs)

        # ---- state / input constraints ----
        for k in range(1, N + 1):
            qi = self._xi(k)
            for a in range(3):
                add({qi + 7 + a: 1.0}, -dmax[a], dmax[a])
            for a in range(4):
                add({qi + 14 + a: 1.0}, -gdmax, gdmax)
            # singularity floor (linearised in gamma)
            g_ref = x_seq[k][10:14]
            sig = self.cluster.sigma_min(self.cluster.jacobian(g_ref))
            grad = self.cluster.sigma_min_grad(g_ref)
            coeff = {self._si(k - 1): 1.0}
            for a in range(4):
                coeff[qi + 10 + a] = -grad[a]
            add(coeff, self.sigma_floor - (sig - grad @ g_ref), np.inf)
        for k in range(N):
            ui = self._ui(k)
            for a in range(3):
                if k == 0:
                    b0 = x0[7 + a]
                    add({ui + a: 1.0}, -np.inf, ddot[a] * v.tau_s + b0)
                    add({ui + a: -1.0}, -np.inf, ddot[a] * v.tau_s - b0)
                else:
                    add({ui + a: 1.0, self._xi(k) + 7 + a: -1.0}, -np.inf, ddot[a] * v.tau_s)
                    add({ui + a: -1.0, self._xi(k) + 7 + a: 1.0}, -np.inf, ddot[a] * v.tau_s)
                add({ui + a: 1.0}, -dmax[a], dmax[a])
            for a in range(4):
                add({ui + 3 + a: 1.0}, -gdmax, gdmax)
                if k == 0:
                    g0 = x0[14 + a]
                    add({ui + 3 + a: 1.0}, -np.inf, gddmax * c.tau_g + g0)
                    add({ui + 3 + a: -1.0}, -np.inf, gddmax * c.tau_g - g0)
                else:
                    add({ui + 3 + a: 1.0, self._xi(k) + 14 + a: -1.0}, -np.inf, gddmax * c.tau_g)
                    add({ui + 3 + a: -1.0, self._xi(k) + 14 + a: 1.0}, -np.inf, gddmax * c.tau_g)
        for k in range(N):
            add({self._si(k): 1.0}, 0.0, np.inf)

        A = np.zeros((len(rows), nz))
        for r, coeff in enumerate(rows):
            for cc, val in coeff.items():
                A[r, cc] = val
        return H, g, A, np.array(lbs), np.array(ubs)

    # ---- helpers --------------------------------------------------------
    def _clamp_prediction(self, z):
        v, c = self.prm.veh, self.prm.cmg
        N = self.N
        X = z[:NX * N].reshape(N, NX).copy()
        U = z[NX * N:(NX + NU) * N].reshape(N, NU).copy()
        for k in range(N):
            X[k, 0:4] = X[k, 0:4] / max(np.linalg.norm(X[k, 0:4]), 1e-6)
        X[:, 4:7] = np.clip(X[:, 4:7], -20.0, 20.0)
        X[:, 7:10] = np.clip(X[:, 7:10], -v.delta_max, v.delta_max)
        X[:, 14:18] = np.clip(X[:, 14:18], -c.gdot_max, c.gdot_max)
        U[:, 0:3] = np.clip(U[:, 0:3], -v.delta_max, v.delta_max)
        U[:, 3:7] = np.clip(U[:, 3:7], -c.gdot_max, c.gdot_max)
        return np.concatenate([X.reshape(-1), U.reshape(-1), np.zeros(N)])

    def _pack(self, u_seq, x_seq):
        return np.concatenate([x_seq[1:].reshape(-1), u_seq.reshape(-1), np.zeros(self.N)])

    def _shift_warm(self, z):
        if z is None:
            return None
        N = self.N
        X = z[:NX * N].reshape(N, NX)
        U = z[NX * N:(NX + NU) * N].reshape(N, NU)
        S = z[(NX + NU) * N:(NX + NU + 1) * N]
        return np.concatenate([np.vstack([X[1:], X[-1:]]).reshape(-1),
                               np.vstack([U[1:], U[-1:]]).reshape(-1),
                               np.concatenate([S[1:], S[-1:]])])

    # ---- solve ----------------------------------------------------------
    def solve(self, x0, t0, traj):
        t_start = time.perf_counter()
        x0 = np.array(x0, float)
        t_seq = np.concatenate([[t0], t0 + np.cumsum(self.dts)])
        p_seq = [traj.p_array(t_seq[k], self.prm.veh) for k in range(self.N)]

        if self.u_seq is None:
            u_seq = self._pd_sequence(x0, traj, t_seq)
            x_seq = self._initial_rollout(x0, p_seq, u_seq)
        else:
            u_seq = np.vstack([self.u_seq[1:], self.u_seq[-1:]])
            x_seq = np.vstack([self.x_seq[1:], self.x_seq[-1:]])
        u_seq = np.array([limit_commands6c(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])

        z = self._shift_warm(self.z_warm)
        viol = 0.0
        failed = False
        n_iter = self.n_sqp if self.z_warm is not None else max(self.n_sqp, 2)
        for _ in range(max(1, n_iter)):
            H, g, A, lb, ub = self._build_qp(x0, x_seq, u_seq, p_seq, t_seq, traj)
            Hs = H + 1e-4 * np.eye(H.shape[0])
            z_raw = self.qp.solve(Hs, g, A, lb, ub, z0=z)
            Az = A @ z_raw
            dv = np.maximum(lb - Az, Az - ub)
            dv = dv[np.isfinite(dv)]
            viol = float(dv.max()) if dv.size else 0.0
            if ("solved" not in self.qp.last_status) or (not np.isfinite(viol)) or viol > 1e-4:
                failed = True
                break
            z = self._clamp_prediction(z_raw)
            x_seq = np.vstack([x0, z[:NX * self.N].reshape(self.N, NX)])
            u_seq = z[NX * self.N:(NX + NU) * self.N].reshape(self.N, NU)
            u_seq = np.array([limit_commands6c(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])
        if failed:
            u_seq = self._pd_sequence(x0, traj, t_seq, u_guess=u_seq)
            u_seq = np.array([limit_commands6c(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])
            x_seq = self._initial_rollout(x0, p_seq, u_seq)
            z = self._pack(u_seq, x_seq)

        self.x_seq = x_seq
        self.u_seq = u_seq
        self.z_warm = z
        u0 = limit_commands6c(x0, u_seq[0], self.prm)
        self.u_prev = u0.copy()
        self.att_err = float(np.linalg.norm(self._attitude_error(traj.q_ref(t0), x0[0:4])))
        sig = [self.cluster.sigma_min(self.cluster.jacobian(x_seq[k][10:14])) for k in range(1, self.N + 1)]
        self.min_sigma = float(np.min(sig))
        self.last_viol = viol
        self.last_solve_time = time.perf_counter() - t_start
        return u0, dict(solve_time=self.last_solve_time, qp_status=self.qp.last_status,
                        viol=viol, att_err=self.att_err, min_sigma=self.min_sigma,
                        H_cmg=float(np.linalg.norm(self.cluster.momentum(x0[10:14]))))
