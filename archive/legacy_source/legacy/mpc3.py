"""Three-axis unified LTV-MPC (successive linearisation + OSQP).

Decision variables: u = [delta_c (3), tau_c (3)] -- the controller commands the
three aerodynamic surfaces and the three wheel motor torques directly, with no
torque-allocation layer.

Attitude error uses the exact vector part of the error quaternion
    e(q) = 2 * vec( q_ref^-1 (x) q ) = 2 * ( w_r q_v - q_w v_r - v_r x q_v )
which is linear in the quaternion state and therefore QP friendly.
"""
from __future__ import annotations

import time

import numpy as np

from nearspace.model6 import (NU, NX, VehicleParams, cross_matrix, discretize3,
                     dynamics3, limit_commands3, plant_step3, quat_conj,
                     quat_mul, wheel_avail3)
from legacy.mpc import QPSolver


class LTVMPC3:
    def __init__(self, prm: VehicleParams, cfg):
        self.prm = prm
        self.cfg = cfg
        m = cfg.mpc3
        self.dts = np.concatenate([np.full(int(seg.n), float(seg.dt)) for seg in m.horizon])
        self.N = int(len(self.dts))
        self.wheel_on = bool(prm.wheel_on)
        self.Q_att = float(m.q_att) * np.eye(3)
        self.Q_rate = float(m.q_rate) * np.eye(3)
        self.W_h = float(m.w_h) * np.eye(3)
        self.R_delta = float(m.r_delta) * np.eye(3)
        self.R_tau = (float(m.r_tau) if self.wheel_on else 0.0) * np.eye(3)
        self.s_delta = float(m.s_delta)
        self.s_tau = float(m.s_tau)
        self.term = float(m.terminal_scale)
        self.n_sqp = int(m.n_sqp)
        self.qp = QPSolver(max_iter=int(getattr(m, "qp_max_iter", 20000)),
                           eps_abs=1e-5, eps_rel=1e-5, internal_scaling=False)
        self.k_att = float(getattr(m, "k_att", 4.0))
        self.k_rate = float(getattr(m, "k_rate", 4.0))
        x_scale = np.concatenate([np.ones(4), 0.01 * np.ones(3), 0.01 * np.ones(3),
                                  np.ones(3), 10.0 * np.ones(3)])
        u_scale = np.concatenate([0.01 * np.ones(3), np.ones(3)])
        self.scales = np.concatenate([np.tile(x_scale, self.N), np.tile(u_scale, self.N)])
        self.u_prev = np.zeros(NU)
        self.x_seq = None
        self.u_seq = None
        self.z_warm = None
        self.last_solve_time = 0.0
        self.last_viol = 0.0
        self.att_err = 0.0

    # ---- indices --------------------------------------------------------
    def _xi(self, k):
        return NX * (k - 1)

    def _ui(self, k):
        return NX * self.N + NU * k

    # ---- attitude error linearisation ----------------------------------
    @staticmethod
    def _err_map(q_ref):
        """E (3x4) such that e = E @ q is 2*vec(q_ref^-1 (x) q)."""
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

    # ---- initial guess --------------------------------------------------
    def _initial_rollout(self, x0, p_seq, u_seq):
        xs = [np.array(x0, dtype=float)]
        x = np.array(x0, dtype=float)
        for u, p, dt in zip(u_seq, p_seq, self.dts):
            n = max(1, int(np.ceil(dt / 0.01)))
            for _ in range(n):
                x, _ = plant_step3(x, u, p, dt / n, self.prm, substeps=1)
            xs.append(x.copy())
        return np.array(xs)

    def _pd_sequence(self, x0, traj, t_seq, u_guess=None):
        """Quaternion PD + aero inversion with the residual on the wheels.

        Used both as the first SQP initial guess and as a failsafe when the QP
        cannot be solved (large errors / rarefied air)."""
        prm = self.prm
        N = self.N
        us = np.zeros((N, NU)) if u_guess is None else np.array(u_guess, dtype=float)
        for k in range(N):
            t = t_seq[k]
            e = self._attitude_error(traj.q_ref(t), x0[0:4])
            w_err = x0[4:7] - traj.omega_ref(t)
            M = np.diag([prm.Jx, prm.Jy, prm.Jz]) @ (self.k_att * e + self.k_rate * w_err)
            Q = 0.5 * traj.rho(t) * traj.V(t) ** 2
            alpha_ref = traj.pitch_ref(t) - traj.gamma(t)
            Ma_cap = Q * prm.S * np.array([prm.b * prm.Cl_da,
                                           prm.cbar * prm.Cm_de,
                                           prm.b * prm.Cn_dr])
            M_rest = np.array([0.0,
                               Q * prm.S * prm.cbar
                               * (prm.Cm0 + prm.Cm_alpha * alpha_ref),
                               0.0])
            denom = np.where(np.abs(Ma_cap) > 1e-9, Ma_cap, 1e-9)
            # moment order is [roll, pitch, yaw] = [aileron, elevator, rudder]
            d_moment = (M - M_rest) / denom
            # state order is [de, da, dr]
            d_state = np.clip(np.array([d_moment[1], d_moment[0], d_moment[2]]),
                              -prm.delta_max, prm.delta_max)
            d_moment_clipped = np.array([d_state[1], d_state[0], d_state[2]])
            M_delivered = M_rest + Ma_cap * d_moment_clipped
            tau_c = np.clip(M - M_delivered, -prm.tau_cmd_max, prm.tau_cmd_max)
            if not prm.wheel_on:
                tau_c[:] = 0.0
            us[k, 0:3] = d_state
            us[k, 3:6] = tau_c
        return us

    def _trim_pulse(self, u_seq, x0, traj, t_seq):
        return self._pd_sequence(x0, traj, t_seq, u_guess=u_seq)

    # ---- QP assembly ----------------------------------------------------
    def _build_qp(self, x0, x_seq, u_seq, p_seq, t_seq, traj):
        N, prm = self.N, self.prm
        nz = (NX + NU) * N
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
                if self.wheel_on:
                    H[qi + 13 + a, qi + 13 + a] += 2.0 * self.W_h[a, a] * dt
        for k in range(N):
            dt = self.dts[k]
            ui = self._ui(k)
            for a in range(3):
                H[ui + a, ui + a] += 2.0 * self.R_delta[a, a] * dt
                H[ui + 3 + a, ui + 3 + a] += 2.0 * self.R_tau[a, a] * dt
        for k in range(1, N):
            H[self._ui(k), self._ui(k)] += 2.0 * self.s_delta
            H[self._ui(k), self._ui(k - 1)] += -2.0 * self.s_delta
            H[self._ui(k - 1), self._ui(k - 1)] += 2.0 * self.s_delta
            if self.wheel_on:
                for a in range(3):
                    i0, i1 = self._ui(k) + 3 + a, self._ui(k - 1) + 3 + a
                    H[i0, i0] += 2.0 * self.s_tau
                    H[i0, i1] += -2.0 * self.s_tau
                    H[i1, i1] += 2.0 * self.s_tau
        H[self._ui(0), self._ui(0)] += 2.0 * self.s_delta
        g[self._ui(0)] += -2.0 * self.s_delta * self.u_prev[0]
        if self.wheel_on:
            for a in range(3):
                H[self._ui(0) + 3 + a, self._ui(0) + 3 + a] += 2.0 * self.s_tau
                g[self._ui(0) + 3 + a] += -2.0 * self.s_tau * self.u_prev[3 + a]
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
            if self.wheel_on:
                H[Pi + 13 + a, Pi + 13 + a] += 2.0 * self.term * self.W_h[a, a]

        dmax = prm.delta_max
        ddot = prm.delta_dot_max
        hmax = prm.h_max

        # ---- dynamics + quaternion norm ----
        for k in range(N):
            Ad, Bd, d0 = discretize3(x_seq[k], u_seq[k], p_seq[k], prm, self.dts[k])
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
                add({qi + 13 + a: 1.0}, -hmax, hmax)
            if self.wheel_on:
                h_ref = x_seq[k][13:16]
                m0 = wheel_avail3(h_ref, prm)
                m1 = np.zeros(3)
                for a in range(3):
                    if (prm.P_max * prm.Jw / max(abs(h_ref[a]), 1e-6) < prm.M_stall
                            and abs(h_ref[a]) > 1e-3):
                        m1[a] = -prm.P_max * prm.Jw * np.sign(h_ref[a]) / (h_ref[a] ** 2)
                for a in range(3):
                    ub = m0[a] - m1[a] * h_ref[a]
                    add({qi + 10 + a: 1.0, qi + 13 + a: -m1[a]}, -np.inf, ub)
                    add({qi + 10 + a: -1.0, qi + 13 + a: -m1[a]}, -np.inf, ub)
            else:
                for a in range(3):
                    add({qi + 10 + a: 1.0}, 0.0, 0.0)
                    add({qi + 13 + a: 1.0}, 0.0, 0.0)

        for k in range(N):
            ui = self._ui(k)
            base = x0[7:10] if k == 0 else None
            for a in range(3):
                if k == 0:
                    b0 = base[a]
                    add({ui + a: 1.0}, -np.inf, ddot[a] * prm.tau_s + b0)
                    add({ui + a: -1.0}, -np.inf, ddot[a] * prm.tau_s - b0)
                else:
                    add({ui + a: 1.0, self._xi(k) + 7 + a: -1.0}, -np.inf, ddot[a] * prm.tau_s)
                    add({ui + a: -1.0, self._xi(k) + 7 + a: 1.0}, -np.inf, ddot[a] * prm.tau_s)
                add({ui + a: 1.0}, -dmax[a], dmax[a])
            if self.wheel_on:
                for a in range(3):
                    add({ui + 3 + a: 1.0}, -prm.tau_cmd_max, prm.tau_cmd_max)
                    if k == 0:
                        add({ui + 3 + a: 1.0}, -np.inf,
                            prm.dtau_cmd_max * self.dts[0] + self.u_prev[3 + a])
                        add({ui + 3 + a: -1.0}, -np.inf,
                            prm.dtau_cmd_max * self.dts[0] - self.u_prev[3 + a])
                    else:
                        lim = prm.dtau_cmd_max * self.dts[k]
                        add({ui + 3 + a: 1.0, self._ui(k - 1) + 3 + a: -1.0}, -np.inf, lim)
                        add({ui + 3 + a: -1.0, self._ui(k - 1) + 3 + a: 1.0}, -np.inf, lim)
            else:
                for a in range(3):
                    add({ui + 3 + a: 1.0}, 0.0, 0.0)

        A = np.zeros((len(rows), nz))
        for r, coeff in enumerate(rows):
            for c, val in coeff.items():
                A[r, c] = val
        return H, g, A, np.array(lbs), np.array(ubs)

    # ---- helpers --------------------------------------------------------
    def _clamp_prediction(self, z):
        prm = self.prm
        N = self.N
        X = z[:NX * N].reshape(N, NX).copy()
        U = z[NX * N:(NX + NU) * N].reshape(N, NU).copy()
        for k in range(N):
            X[k, 0:4] = X[k, 0:4] / max(np.linalg.norm(X[k, 0:4]), 1e-6)
        X[:, 4:7] = np.clip(X[:, 4:7], -20.0, 20.0)
        X[:, 7:10] = np.clip(X[:, 7:10], -prm.delta_max, prm.delta_max)
        X[:, 10:13] = np.clip(X[:, 10:13], -2.0 * prm.tau_cmd_max, 2.0 * prm.tau_cmd_max)
        X[:, 13:16] = np.clip(X[:, 13:16], -prm.h_max, prm.h_max)
        U[:, 0:3] = np.clip(U[:, 0:3], -prm.delta_max, prm.delta_max)
        U[:, 3:6] = np.clip(U[:, 3:6], -prm.tau_cmd_max, prm.tau_cmd_max)
        return np.concatenate([X.reshape(-1), U.reshape(-1)])

    @staticmethod
    def _equilibrate(H, A):
        """Jacobi equilibration: unit diagonal in the cost, then row scaling."""
        dh = np.diag(H).copy()
        col = np.max(np.abs(A), axis=0)
        d = np.where(dh > 1e-10, 1.0 / np.sqrt(np.maximum(dh, 1e-30)),
                     1.0 / np.maximum(col, 1e-9))
        return np.clip(d, 1e-2, 1e2)

    def _pack(self, u_seq, x_seq):
        N = self.N
        return np.concatenate([x_seq[1:].reshape(-1), u_seq.reshape(-1)])

    def _shift_warm(self, z):
        if z is None:
            return None
        N = self.N
        X = z[:NX * N].reshape(N, NX)
        U = z[NX * N:(NX + NU) * N].reshape(N, NU)
        return np.concatenate([np.vstack([X[1:], X[-1:]]).reshape(-1),
                               np.vstack([U[1:], U[-1:]]).reshape(-1)])

    # ---- main -----------------------------------------------------------
    def solve(self, x0, t0, traj):
        t_start = time.perf_counter()
        x0 = np.array(x0, dtype=float)
        t_seq = np.concatenate([[t0], t0 + np.cumsum(self.dts)])
        p_seq = [traj.p_array(t_seq[k], self.prm) for k in range(self.N)]

        if self.u_seq is None:
            u_seq = np.zeros((self.N, NU))
            u_seq = self._trim_pulse(u_seq, x0, traj, t_seq)
            x_seq = self._initial_rollout(x0, p_seq, u_seq)
        else:
            u_seq = np.vstack([self.u_seq[1:], self.u_seq[-1:]])
            x_seq = np.vstack([self.x_seq[1:], self.x_seq[-1:]])
        u_seq = np.array([limit_commands3(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])

        z = self._shift_warm(self.z_warm)
        viol = 0.0
        failed = False
        n_iter = self.n_sqp if self.z_warm is not None else max(self.n_sqp, 2)
        for _ in range(max(1, n_iter)):
            H, g, A, lb, ub = self._build_qp(x0, x_seq, u_seq, p_seq, t_seq, traj)
            Hs = H + 1e-4 * np.eye(H.shape[0])   # row scaling is done inside QPSolver
            gs = g
            As = A
            D = np.ones(H.shape[0])
            z0s = None if z is None else z / D
            z_raw = D * self.qp.solve(Hs, gs, As, lb, ub, z0=z0s)
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
            u_seq = np.array([limit_commands3(x_seq[k], u_seq[k], self.prm)
                              for k in range(self.N)])
        if failed:
            # failsafe: quaternion PD + aero inversion, residual on the wheels
            u_seq = self._pd_sequence(x0, traj, t_seq, u_guess=u_seq)
            u_seq = np.array([limit_commands3(x_seq[k], u_seq[k], self.prm)
                              for k in range(self.N)])
            x_seq = self._initial_rollout(x0, p_seq, u_seq)
            z = self._pack(u_seq, x_seq)

        self.x_seq = x_seq
        self.u_seq = u_seq
        self.z_warm = z
        u0 = limit_commands3(x0, u_seq[0], self.prm)
        self.u_prev = u0.copy()
        self.att_err = float(np.linalg.norm(self._attitude_error(traj.q_ref(t0), x0[0:4])))
        self.last_viol = viol
        self.last_solve_time = time.perf_counter() - t_start
        return u0, dict(solve_time=self.last_solve_time, qp_status=self.qp.last_status,
                        viol=viol, att_err=self.att_err,
                        max_h=float(np.max(np.abs(x_seq[:, 13:16]))))
