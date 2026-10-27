"""Aero-surface + CMG unified control (realistic two-layer actuator).

Architecture
    outer : unified MPC commands  u = [delta_c (3), M_cmg (3)]
            - aero surfaces and CMG torque are decided together (no allocation);
            - the prediction model carries the full actuator state
                  x = [q(4), omega(3), delta_aero(3), gamma(4), gamma_dot(4)]  (18)
              with the steering law inside:
                  gamma_dot_cmd = -S(gamma) M_cmg,  S = A^T (A A^T + lam^2 I)^-1
                  d(gamma)/dt   = gamma_dot
                  d(gamma_dot)/dt = (gamma_dot_cmd - gamma_dot)/tau_g   (lag)
                  M_cmg_actual  = -A(gamma) gamma_dot
              so the gimbal-rate limit and the torque actually delivered are
              represented, including the actuator lag;
            - extra deliverable-torque envelope (inner approximation of the
              zonotope {A u : |u|_inf <= gdot_max}) in the left-singular basis:
                  |U[:,i]^T M_cmg| <= k * sigma_i * sqrt(n) * gdot_max
              which automatically collapses near singularities;
            - singularity floor on sigma_min(A) (conservative linearisation +
              slack penalty).
    inner : adaptive damped pseudo-inverse steering law with null motion toward
            the home configuration (what real CMGs run).
"""
from __future__ import annotations

import time

import numpy as np
from scipy.linalg import expm

from .cmg import CMGCluster
from .model6 import cross_matrix, quat_conj, quat_mul
from .model6c import VehicleCMGParams, aero_moment6c, limit_commands6c
from .qp import QPSolver

NXP, NUP = 18, 6


# --------------------------------------------------------------------------
# steering law
# --------------------------------------------------------------------------
def steering_matrix(cluster: CMGCluster, gamma, lam_scale=0.03):
    A = cluster.jacobian(gamma)
    sig = cluster.sigma_min(A)
    lam = max(lam_scale * cluster.prm.h0 * max(0.0, 1.0 - sig / (2.0 * cluster.prm.sigma_floor)), 1e-3)
    S = A.T @ np.linalg.inv(A @ A.T + lam ** 2 * np.eye(3))
    return A, S


def steering_law(cluster: CMGCluster, gamma, M_cmd, gamma_home, k_null=0.5, lam_scale=0.03):
    """Singularity-robust steering law.

    Torque-producing part: damped pseudo-inverse.
    Null motion: uphill on det(A A^T) (moves away from the singular surface)
    projected on the null space; if that gradient vanishes it falls back to
    pulling the gimbals toward the home configuration. Without this, a constant
    torque command drives the pyramid into a singularity within a few seconds.
    """
    A, S = steering_matrix(cluster, gamma, lam_scale)
    gdot = -S @ M_cmd
    lam = max(lam_scale * cluster.prm.h0, 1e-6)
    pinvA = A.T @ np.linalg.inv(A @ A.T + lam ** 2 * np.eye(3))
    N = np.eye(cluster.prm.n) - pinvA @ A
    grad = cluster.singularity_grad(gamma)
    null = N @ grad
    ng = float(np.linalg.norm(grad))
    nn = float(np.linalg.norm(null))
    if ng > 1e-9 and nn > 0.05 * ng:
        # scale by the null-space share: no amplification of numerical noise
        # (at a symmetric configuration N*grad ~ 0 and normalising it would
        # command a full-magnitude rate that produces a spurious torque)
        gdot = gdot + k_null * cluster.prm.gdot_max * null / ng
    else:
        back = N @ (gamma_home - gamma)
        nb = np.linalg.norm(back)
        db = np.linalg.norm(gamma_home - gamma)
        if nb > 1e-6 and db > 1e-9:
            gdot = gdot + k_null * cluster.prm.gdot_max * back / db
    return np.clip(gdot, -cluster.prm.gdot_max, cluster.prm.gdot_max)


# --------------------------------------------------------------------------
# prediction model (18 states, input = [delta_c, M_cmg])
# --------------------------------------------------------------------------
def dynamics_eff(x, u, p, prm: VehicleCMGParams, cluster: CMGCluster):
    v = prm.veh
    q, w, da, gamma, gdot = x[0:4], x[4:7], x[7:10], x[10:14], x[14:18]
    dx = np.empty(NXP)
    dx[0:4] = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
    if prm.cmg_on:
        A, S = steering_matrix(cluster, gamma)
        gdot_cmd = -S @ u[3:6]
        Hc = cluster.momentum(gamma)
        Mc = -A @ gdot                       # torque actually delivered (lagged rate)
        dx[10:14] = gdot
        dx[14:18] = (gdot_cmd - gdot) / prm.cmg.tau_g
    else:                                    # mode A: no CMG in the model at all
        Hc = np.zeros(3)
        Mc = np.zeros(3)
        dx[10:14] = 0.0
        dx[14:18] = 0.0
    xa = np.concatenate([x[0:7], da, np.zeros(3), np.zeros(3)])
    Mtot = aero_moment6c(xa, p, prm) + p[4:7] + Mc
    dx[4:7] = prm.Jinv @ (-np.cross(w, prm.J @ w + Hc) + Mtot)
    dx[7:10] = (u[0:3] - da) / v.tau_s
    return dx


def jacobian_eff(x, u, p, prm, cluster):
    A = np.zeros((NXP, NXP)); B = np.zeros((NXP, NUP))
    for i in range(NXP):
        h = 1e-6 * max(1.0, abs(x[i]))
        xp = x.copy(); xp[i] += h
        xm = x.copy(); xm[i] -= h
        A[:, i] = (dynamics_eff(xp, u, p, prm, cluster)
                   - dynamics_eff(xm, u, p, prm, cluster)) / (2 * h)
    for j in range(NUP):
        h = 1e-6 * max(1.0, abs(u[j]))
        up = u.copy(); up[j] += h
        um = u.copy(); um[j] -= h
        B[:, j] = (dynamics_eff(x, up, p, prm, cluster)
                   - dynamics_eff(x, um, p, prm, cluster)) / (2 * h)
    return A, B


def discretize_eff(x, u, p, prm, cluster, dt):
    A, B = jacobian_eff(x, u, p, prm, cluster)
    f0 = dynamics_eff(x, u, p, prm, cluster) - A @ x - B @ u
    M = np.zeros((NXP + 1 + NUP, NXP + 1 + NUP))
    M[:NXP, :NXP] = A; M[:NXP, NXP] = f0; M[:NXP, NXP + 1:] = B
    E = expm(M * dt)
    return E[:NXP, :NXP], E[:NXP, NXP + 1:], E[:NXP, NXP]


# --------------------------------------------------------------------------
# unified MPC
# --------------------------------------------------------------------------
class LTVMPCCMG2:
    def __init__(self, prm: VehicleCMGParams, cfg, cluster: CMGCluster | None = None):
        self.prm = prm
        self.cfg = cfg
        self.cluster = cluster if cluster is not None else CMGCluster(prm.cmg)
        m = getattr(cfg, "mpc_cmg", None) or cfg.mpc3
        self.dts = np.concatenate([np.full(int(s.n), float(s.dt)) for s in m.horizon])
        self.N = int(len(self.dts))
        self.Q_att = float(m.q_att) * np.eye(3)
        self.Q_rate = float(m.q_rate) * np.eye(3)
        self.R_delta = float(m.r_delta) * np.eye(3)
        self.R_cmg = float(getattr(m, "r_cmg", 1.0e-3)) * np.eye(3)
        self.W_gamma = float(getattr(m, "w_gamma", 5.0))
        self.W_H = float(getattr(m, "w_H", 1.0e-3))
        self.s_delta = float(m.s_delta)
        self.rho_sing = float(getattr(m, "rho_sing", 10000.0))
        self.term = float(m.terminal_scale)
        self.n_sqp = int(m.n_sqp)
        self.k_att = float(getattr(m, "k_att", 4.0))
        self.k_rate = float(getattr(m, "k_rate", 4.0))
        self.k_unload = float(getattr(m, "k_unload", 0.05))
        self.M_box = float(getattr(m, "M_cmg_box", 250.0))
        self.torque_margin = float(getattr(m, "torque_margin", 0.8))
        self.H_lim = float(getattr(m, "H_lim_frac", 0.35)) * prm.cmg.h_total
        self.q_unload_min = float(getattr(m, "q_unload_min", 40.0))
        # fraction of M_box the controller may demand purely for unloading;
        # dumping momentum always reacts on the vehicle, so it is rate-limited
        self.unload_cap = float(getattr(m, "unload_cap", 0.6))
        self.viol_tol = float(getattr(m, "qp_viol_tol", 1.0e-3))
        self.r_cmg_tr = float(getattr(m, "r_cmg_transient", 0.01))
        self.r_cmg_trim = float(getattr(m, "r_cmg_trim", 0.5))
        self.e_thresh = np.deg2rad(float(getattr(m, "e_thresh_deg", 0.5)))
        self.sing_margin = float(getattr(m, "sigma_margin", 1.15))
        self.gamma_home = np.zeros(prm.cmg.n)
        self.qp = QPSolver(max_iter=int(getattr(m, "qp_max_iter", 20000)),
                           eps_abs=1e-5, eps_rel=1e-5, internal_scaling=False)
        self.u_prev = np.zeros(NUP)
        self.x_seq = None; self.u_seq = None; self.z_warm = None
        self.last_solve_time = 0.0; self.last_viol = 0.0
        self.att_err = 0.0; self.min_sigma = 0.0

    def _xi(self, k): return NXP * (k - 1)
    def _ui(self, k): return NXP * self.N + NUP * k
    def _si(self, k): return NXP * self.N + NUP * self.N + k

    @staticmethod
    def _err_map(qr):
        w_r, v_r = qr[0], qr[1:4]
        E = np.zeros((3, 4)); E[:, 0] = -2.0 * v_r
        E[:, 1:4] = 2.0 * (w_r * np.eye(3) - cross_matrix(v_r))
        return E

    @staticmethod
    def _att_error(qr, q):
        dq = quat_mul(quat_conj(qr), q)
        if dq[0] < 0.0:
            dq = -dq
        return 2.0 * dq[1:4]

    def command_from_x(self, x):
        return np.asarray(x, dtype=float)[:NXP]

    # ---- failsafe --------------------------------------------------------
    def failsafe(self, x, t, traj):
        v, c = self.prm.veh, self.prm.cmg
        e = self._att_error(traj.q_ref(t), x[0:4])
        w_err = x[4:7] - traj.omega_ref(t)
        J = np.diag([v.Jx, v.Jy, v.Jz])
        M_des = J @ (self.k_att * e + self.k_rate * w_err)
        Hc = self.cluster.momentum(x[10:14])
        # classical split: CMG = damping (rate feedback) only, aero = stiffness;
        # prevents the failsafe from charging the CMG to its momentum budget
        M_cmd = np.clip(J @ (self.k_rate * w_err), -self.M_box, self.M_box)
        nH = float(np.linalg.norm(Hc))
        if nH > self.H_lim and nH > 1e-9:          # same momentum budget as the QP
            u_hat = Hc / nH
            comp = float(np.dot(M_cmd, u_hat))
            if comp < 0.0:
                M_cmd = M_cmd - comp * u_hat
        gdot = steering_law(self.cluster, x[10:14], M_cmd, self.gamma_home)
        if self.prm.cmg_on:
            M_cmg_actual = -self.cluster.jacobian(x[10:14]) @ gdot
        else:
            gdot = np.zeros(c.n)
            M_cmg_actual = np.zeros(3)
        M_res = M_des - M_cmg_actual + self.k_unload * Hc
        Q = 0.5 * traj.rho(t) * traj.V(t) ** 2
        alpha_ref = traj.pitch_ref(t) - traj.gamma(t)
        cap = Q * v.S * np.array([v.b * v.Cl_da, v.cbar * v.Cm_de, v.b * v.Cn_dr])
        rest = np.array([0.0, Q * v.S * v.cbar * (v.Cm0 + v.Cm_alpha * alpha_ref), 0.0])
        den = np.where(np.abs(cap) > 1e-9, cap, 1e-9)
        d_m = (M_res - rest) / den
        d_state = np.clip(np.array([d_m[1], d_m[0], d_m[2]]), -v.delta_max, v.delta_max)
        u = np.concatenate([d_state, gdot])
        return limit_commands6c(x, u, self.prm)

    # ---- QP --------------------------------------------------------------
    def _build_qp(self, x0, x_seq, u_seq, p_seq, t_seq, traj):
        N = self.N
        v, c = self.prm.veh, self.prm.cmg
        nz = (NXP + NUP + 1) * N
        H = np.zeros((nz, nz)); g = np.zeros(nz)
        rows, lbs, ubs = [], [], []

        def add(coeffs, lb, ub):
            rows.append(coeffs); lbs.append(lb); ubs.append(ub)

        for k in range(1, N + 1):
            dt = self.dts[k - 1]; t = t_seq[k]
            E = self._err_map(traj.q_ref(t)); He = E.T @ self.Q_att @ E
            qi = self._xi(k)
            for a in range(4):
                for b in range(4):
                    H[qi + a, qi + b] += 2.0 * He[a, b] * dt
            w_ref = traj.omega_ref(t)
            for a in range(3):
                H[qi + 4 + a, qi + 4 + a] += 2.0 * self.Q_rate[a, a] * dt
                g[qi + 4 + a] += -2.0 * self.Q_rate[a, a] * dt * w_ref[a]
            for a in range(4):
                H[qi + 10 + a, qi + 10 + a] += 2.0 * self.W_gamma * dt
            Href = self.cluster.momentum(x_seq[k][10:14])
            Aref = self.cluster.jacobian(x_seq[k][10:14])
            for a in range(4):
                for b in range(4):
                    H[qi + 10 + a, qi + 10 + b] += 2.0 * self.W_H * dt * (Aref[:, a] @ Aref[:, b])
                g[qi + 10 + a] += 2.0 * self.W_H * dt * (Aref[:, a] @ (Href - Aref @ x_seq[k][10:14]))
        for k in range(N):
            dt = self.dts[k]; ui = self._ui(k)
            # error-adaptive CMG effort: cheap during transients (CMG fires),
            # expensive when trimmed (aero carries the sustained torque)
            ek = np.linalg.norm(self._att_error(traj.q_ref(t_seq[k]), x_seq[k][0:4]))
            r_cmg = (self.r_cmg_tr if ek > self.e_thresh else self.r_cmg_trim)
            for a in range(3):
                H[ui + a, ui + a] += 2.0 * self.R_delta[a, a] * dt
                H[ui + 3 + a, ui + 3 + a] += 2.0 * r_cmg * dt
        for k in range(1, N):
            H[self._ui(k), self._ui(k)] += 2.0 * self.s_delta
            H[self._ui(k), self._ui(k - 1)] += -2.0 * self.s_delta
            H[self._ui(k - 1), self._ui(k - 1)] += 2.0 * self.s_delta
        H[self._ui(0), self._ui(0)] += 2.0 * self.s_delta
        g[self._ui(0)] += -2.0 * self.s_delta * self.u_prev[0]
        for k in range(N):
            H[self._si(k), self._si(k)] += 2.0 * self.rho_sing
        tN = t_seq[N]; E = self._err_map(traj.q_ref(tN))
        He = self.term * (E.T @ self.Q_att @ E); Pi = self._xi(N)
        for a in range(4):
            for b in range(4):
                H[Pi + a, Pi + b] += 2.0 * He[a, b]
        w_ref = traj.omega_ref(tN)
        for a in range(3):
            H[Pi + 4 + a, Pi + 4 + a] += 2.0 * self.term * self.Q_rate[a, a]
            g[Pi + 4 + a] += -2.0 * self.term * self.Q_rate[a, a] * w_ref[a]

        dmax, ddot = v.delta_max, v.delta_dot_max

        for k in range(N):
            Ad, Bd, d0 = discretize_eff(x_seq[k], u_seq[k], p_seq[k], self.prm, self.cluster, self.dts[k])
            for i in range(NXP):
                coeff = {self._xi(k + 1) + i: 1.0}
                if k == 0:
                    rhs = d0[i] + (Ad @ x0)[i]
                else:
                    for j in range(NXP):
                        if Ad[i, j] != 0.0:
                            coeff[self._xi(k) + j] = coeff.get(self._xi(k) + j, 0.0) - Ad[i, j]
                    rhs = d0[i]
                for j in range(NUP):
                    if Bd[i, j] != 0.0:
                        coeff[self._ui(k) + j] = coeff.get(self._ui(k) + j, 0.0) - Bd[i, j]
                add(coeff, rhs, rhs)

        for k in range(1, N + 1):
            qi = self._xi(k)
            for a in range(3):
                add({qi + 7 + a: 1.0}, -dmax[a], dmax[a])
            for a in range(4):
                add({qi + 14 + a: 1.0}, -c.gdot_max, c.gdot_max)
            g_ref = x_seq[k][10:14]
            Aref = self.cluster.jacobian(g_ref)
            sig = self.cluster.sigma_min(Aref)
            grad = self.cluster.sigma_min_grad(g_ref)
            coeff = {self._si(k - 1): 1.0}
            for a in range(4):
                coeff[qi + 10 + a] = -grad[a]
            # conservative: require sigma >= margin * floor along the linearisation
            add(coeff, self.sing_margin * self.sigma_floor - (sig - grad @ g_ref), np.inf)
            # CMG momentum budget.  A component-wise *hard* box cannot be the
            # primary limit: once |H| is over budget (which one node step of
            # gimbal rate cannot undo) such a box is infeasible for every
            # direction and the QP dies permanently.  So the box is relaxed to
            # the current value -- "hold" is always reachable -- and the real
            # limit is enforced radially on the torque, just below.
            Href = self.cluster.momentum(g_ref)
            base = Href - Aref @ g_ref
            Hc = self.H_lim / np.sqrt(3.0)
            for i in range(3):
                lo = min(-Hc, float(Href[i])); hi = max(Hc, float(Href[i]))
                add({qi + 10 + a: float(Aref[i, a]) for a in range(4)},
                    lo - base[i], hi - base[i])

        for k in range(N):
            ui = self._ui(k)
            g_ref = x_seq[k][10:14]
            Aref = self.cluster.jacobian(g_ref)
            _, S = steering_matrix(self.cluster, g_ref)
            U, sv, _ = np.linalg.svd(Aref)
            for i in range(4):                       # exact (linearised) rate limit
                add({ui + 3 + j: float(S[i, j]) for j in range(3)}, -c.gdot_max, c.gdot_max)
            for i in range(3):                       # deliverable torque envelope
                cap = self.torque_margin * sv[i] * np.sqrt(c.n) * c.gdot_max
                add({ui + 3 + j: float(U[j, i]) for j in range(3)}, -cap, cap)
            for j in range(3):
                add({ui + 3 + j: 1.0}, -self.M_box, self.M_box)
            # predictive momentum management, radial form:
            #   d|H|^2/dt = -2 H . M_cmg
            #   =>  H . M_cmg >= (|H|^2 - H_lim^2) / (2 dt)
            # Under budget this bounds how fast the CMG may charge; over budget
            # it asks for active unloading.  Because it constrains only the
            # component *along* H, a saturated cluster keeps its full
            # perpendicular authority -- that is what the component-wise box
            # destroyed.  The per-node slack keeps the row feasible.
            Hk = self.cluster.momentum(x_seq[k][10:14])
            nHk = float(np.linalg.norm(Hk))
            rhs = (nHk * nHk - self.H_lim ** 2) / (2.0 * max(self.dts[k], 1e-6))
            rhs = min(rhs, self.unload_cap * self.M_box)
            coeff = {ui + 3 + j: float(Hk[j]) for j in range(3)}
            coeff[self._si(k)] = 1.0
            add(coeff, rhs, np.inf)
            # in thin air the aero surfaces cannot absorb what the CMG dumps,
            # so unloading is only permitted while the budget still holds
            qk = 0.5 * p_seq[k][0] * float(np.dot(p_seq[k][1:4], p_seq[k][1:4]))
            if qk < self.q_unload_min and 1e-6 < nHk <= self.H_lim:
                add({ui + 3 + j: float(Hk[j]) for j in range(3)}, -np.inf, 0.0)
            for a in range(3):
                if k == 0:
                    b0 = x0[7 + a]
                    add({ui + a: 1.0}, -np.inf, ddot[a] * v.tau_s + b0)
                    add({ui + a: -1.0}, -np.inf, ddot[a] * v.tau_s - b0)
                else:
                    add({ui + a: 1.0, self._xi(k) + 7 + a: -1.0}, -np.inf, ddot[a] * v.tau_s)
                    add({ui + a: -1.0, self._xi(k) + 7 + a: 1.0}, -np.inf, ddot[a] * v.tau_s)
                add({ui + a: 1.0}, -dmax[a], dmax[a])
        for k in range(N):
            add({self._si(k): 1.0}, 0.0, np.inf)

        A = np.zeros((len(rows), nz))
        for r, coeff in enumerate(rows):
            for cc, val in coeff.items():
                A[r, cc] = val
        return H, g, A, np.array(lbs), np.array(ubs)

    @property
    def sigma_floor(self):
        return self.prm.cmg.sigma_floor

    # ---- solve -----------------------------------------------------------
    def solve(self, x_plant, t0, traj):
        t_start = time.perf_counter()
        x0 = self.command_from_x(np.asarray(x_plant, float))
        t_seq = np.concatenate([[t0], t0 + np.cumsum(self.dts)])
        p_seq = [traj.p_array(t_seq[k], self.prm.veh) for k in range(self.N)]

        if self.u_seq is None:
            u_seq = np.zeros((self.N, NUP))
            x_seq = self._rollout(x0, p_seq, u_seq)
        else:
            u_seq = np.vstack([self.u_seq[1:], self.u_seq[-1:]])
            x_seq = np.vstack([self.x_seq[1:], self.x_seq[-1:]])
        z = self._shift_warm(self.z_warm)
        viol = 0.0; failed = False
        n_iter = self.n_sqp if self.z_warm is not None else max(self.n_sqp, 3)
        for _ in range(max(1, n_iter)):
            H, g, A, lb, ub = self._build_qp(x0, x_seq, u_seq, p_seq, t_seq, traj)
            Hs = H + 1e-4 * np.eye(H.shape[0])
            z_raw = self.qp.solve(Hs, g, A, lb, ub, z0=z)
            Az = A @ z_raw
            dv = np.maximum(lb - Az, Az - ub); dv = dv[np.isfinite(dv)]
            viol = float(dv.max()) if dv.size else 0.0
            # The constraint violation is the authority on success, not the OSQP
            # status string: an "inaccurate" solve with a tiny violation is a
            # perfectly usable command (implementation plan §10.8 of the status
            # report / §2.3-B).  Only a real violation triggers the fallback.
            if (not np.isfinite(viol)) or viol > self.viol_tol:
                failed = True; break
            z = self._clamp(z_raw)
            x_seq = np.vstack([x0, z[:NXP * self.N].reshape(self.N, NXP)])
            u_seq = z[NXP * self.N:(NXP + NUP) * self.N].reshape(self.N, NUP)
        if failed:
            u = self.failsafe(x_plant, t0, traj)
            return u, dict(solve_time=time.perf_counter() - t_start,
                           qp_status=self.qp.last_status, qp_iter=self.qp.last_iter,
                           fallback=1, viol=viol,
                           att_err=float(np.linalg.norm(self._att_error(traj.q_ref(t0), x_plant[0:4]))),
                           min_sigma=self.cluster.sigma_min(self.cluster.jacobian(x_plant[10:14])),
                           H_cmg=float(np.linalg.norm(self.cluster.momentum(x_plant[10:14]))))

        self.x_seq = x_seq; self.u_seq = u_seq; self.z_warm = z
        M_cmg = u_seq[0][3:6]
        # momentum manager: above the budget only allow commands that do not
        # increase |H| (dH/dt = -M, so growing requires dot(M,H) < 0)
        H = self.cluster.momentum(x_plant[10:14])
        nH = float(np.linalg.norm(H))
        if nH > self.H_lim and nH > 1e-9:
            u_hat = H / nH
            comp = float(np.dot(M_cmg, u_hat))
            if comp < 0.0:
                M_cmg = M_cmg - comp * u_hat
        gdot_cmd = steering_law(self.cluster, x_plant[10:14], M_cmg, self.gamma_home)
        u = limit_commands6c(x_plant, np.concatenate([u_seq[0][0:3], gdot_cmd]), self.prm)
        self.u_prev = np.concatenate([u_seq[0][0:3], M_cmg])
        self.att_err = float(np.linalg.norm(self._att_error(traj.q_ref(t0), x_plant[0:4])))
        sig = [self.cluster.sigma_min(self.cluster.jacobian(x_seq[k][10:14])) for k in range(1, self.N + 1)]
        self.min_sigma = float(np.min(sig))
        self.last_viol = viol
        self.last_solve_time = time.perf_counter() - t_start
        return u, dict(solve_time=self.last_solve_time, qp_status=self.qp.last_status,
                       qp_iter=self.qp.last_iter, fallback=0,
                       viol=viol, att_err=self.att_err, min_sigma=self.min_sigma,
                       H_cmg=float(np.linalg.norm(self.cluster.momentum(x_plant[10:14]))))

    # ---- helpers ---------------------------------------------------------
    def _rollout(self, x0, p_seq, u_seq):
        xs = [np.array(x0, float)]; x = np.array(x0, float)
        for u, p, dt in zip(u_seq, p_seq, self.dts):
            n = max(1, int(np.ceil(dt / 0.01)))
            for _ in range(n):
                x = self._rk4(x, u, p, dt / n)
            xs.append(x.copy())
        return np.array(xs)

    def _rk4(self, x, u, p, dt):
        f = lambda z: dynamics_eff(z, u, p, self.prm, self.cluster)
        k1 = f(x); k2 = f(x + 0.5 * dt * k1); k3 = f(x + 0.5 * dt * k2); k4 = f(x + dt * k3)
        x = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        x[0:4] = x[0:4] / max(np.linalg.norm(x[0:4]), 1e-9)
        x[7:10] = np.clip(x[7:10], -self.prm.veh.delta_max, self.prm.veh.delta_max)
        x[14:18] = np.clip(x[14:18], -self.prm.cmg.gdot_max, self.prm.cmg.gdot_max)
        return x

    def _clamp(self, z):
        N = self.N
        X = z[:NXP * N].reshape(N, NXP).copy()
        U = z[NXP * N:(NXP + NUP) * N].reshape(N, NUP).copy()
        for k in range(N):
            X[k, 0:4] /= max(np.linalg.norm(X[k, 0:4]), 1e-6)
        X[:, 4:7] = np.clip(X[:, 4:7], -20, 20)
        X[:, 7:10] = np.clip(X[:, 7:10], -self.prm.veh.delta_max, self.prm.veh.delta_max)
        X[:, 14:18] = np.clip(X[:, 14:18], -self.prm.cmg.gdot_max, self.prm.cmg.gdot_max)
        U[:, 0:3] = np.clip(U[:, 0:3], -self.prm.veh.delta_max, self.prm.veh.delta_max)
        U[:, 3:6] = np.clip(U[:, 3:6], -self.M_box, self.M_box)
        return np.concatenate([X.reshape(-1), U.reshape(-1), np.zeros(N)])

    def _shift_warm(self, z):
        if z is None:
            return None
        N = self.N
        X = z[:NXP * N].reshape(N, NXP); U = z[NXP * N:(NXP + NUP) * N].reshape(N, NUP)
        S = z[(NXP + NUP) * N:]
        return np.concatenate([np.vstack([X[1:], X[-1:]]).reshape(-1),
                               np.vstack([U[1:], U[-1:]]).reshape(-1),
                               np.concatenate([S[1:], S[-1:]])])
