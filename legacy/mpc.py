"""Unified LTV-MPC for the hybrid actuator problem.

The controller optimises the actuator commands directly:
    u = [delta_c, M_rw_c]
over a non-uniform horizon with

  * attitude tracking, control effort and slew cost,
  * a soft angular-momentum penalty w_h * h_w^2 that keeps the wheel inside the
    region where its torque authority is high (the wheel torque-speed envelope
    |M_rw| <= min(M_stall, P_max J_w/|h_w|) is enforced as a state-dependent
    constraint), so the aero surface is used to manage wheel momentum whenever
    it can,
  * servo position/rate-reachability constraints and motor command limits.

The control-authority margin
    Gamma^pm = M_a,max/min + M_rw,avail(h_w) - M_req
is evaluated along the predicted trajectory and reported as a performance
metric (it is not a hard QP constraint, which keeps the QP always feasible).

The nonlinear model is input-affine with constant B, so successive
linearisation (SQP) reduces each sample to a sparse QP solved by row-scaled
OSQP (ADMM fallback).
"""
from __future__ import annotations

import time
import numpy as np
from scipy.linalg import cho_factor, cho_solve, solve_discrete_are

from legacy.model import (ModelParams, aero_Q, discretize, dynamics,
                    limit_commands, wheel_avail)


# --------------------------------------------------------------------------
# QP solver:  min 0.5 z'H z + g'z   s.t.  l <= A z <= u
# --------------------------------------------------------------------------
try:  # optional dependency
    import osqp as _osqp
    import scipy.sparse as _sp
    _HAVE_OSQP = True
except Exception:  # pragma: no cover
    _HAVE_OSQP = False


class _ADMMFallback:
    def __init__(self, rho: float = 1.0, sigma: float = 1e-5,
                 max_iter: int = 20000, tol: float = 1e-6, alpha: float = 1.6):
        self.rho = rho
        self.sigma = sigma
        self.max_iter = max_iter
        self.tol = tol
        self.alpha = alpha

    def solve(self, H, g, A, l, u, z0=None):
        nz = H.shape[0]
        P = H + self.sigma * np.eye(nz) + self.rho * (A.T @ A)
        P = 0.5 * (P + P.T)
        try:
            factor = cho_factor(P, lower=True, check_finite=False)
        except np.linalg.LinAlgError:
            factor = cho_factor(P + 1e-6 * np.eye(nz), lower=True, check_finite=False)
        At = A.T
        z = np.zeros(nz) if z0 is None else np.array(z0, dtype=float).copy()
        y = np.zeros(A.shape[0])
        v = np.clip(A @ z, l, u)
        for _ in range(self.max_iter):
            z = cho_solve(factor, self.sigma * z - g + self.rho * (At @ (v - y)), check_finite=False)
            Az = A @ z
            v_new = np.clip(self.alpha * Az + (1.0 - self.alpha) * v + y, l, u)
            y = y + self.alpha * Az + (1.0 - self.alpha) * v - v_new
            v = v_new
        return z


class QPSolver:
    """Row-scaled OSQP (fast) with ADMM fallback; stable call signature."""

    def __init__(self, max_iter: int = 20000, eps_abs: float = 1e-7,
                 eps_rel: float = 1e-7, use_osqp: bool = True,
                 internal_scaling: bool = True):
        self.max_iter = max_iter
        self.eps_abs = eps_abs
        self.eps_rel = eps_rel
        self.internal_scaling = bool(internal_scaling)
        self.use_osqp = bool(use_osqp and _HAVE_OSQP)
        self.fallback = _ADMMFallback()
        self.last_status = "not_solved"
        self.last_iter = 0

    def solve(self, H, g, A, l, u, z0=None):
        scale = np.max(np.abs(A), axis=1)
        scale = np.where(scale > 0.0, scale, 1.0)
        As = A / scale[:, None]
        ls = np.where(np.isfinite(l), l, -1e30) / scale
        us = np.where(np.isfinite(u), u, 1e30) / scale
        ls = np.clip(ls, -1e30, 1e30)
        us = np.clip(us, -1e30, 1e30)
        if self.use_osqp and np.any(ls > us + 1e-9):  # invalid bounds -> fallback
            self.use_osqp = False
        if self.use_osqp:
            try:
                P = _sp.triu(_sp.csc_matrix(H)).tocsc()
                prob = _osqp.OSQP()
                prob.setup(P=P, q=np.asarray(g, dtype=float), A=_sp.csc_matrix(As),
                           l=np.asarray(ls, dtype=float), u=np.asarray(us, dtype=float),
                           verbose=False, eps_abs=self.eps_abs, eps_rel=self.eps_rel,
                           max_iter=self.max_iter, polish=True, warm_starting=True,
                           scaling=(1 if self.internal_scaling else 0))
                if z0 is not None:
                    prob.warm_start(x=np.asarray(z0, dtype=float), y=np.zeros(A.shape[0]))
                res = prob.solve()
                self.last_status = str(res.info.status)
                self.last_iter = int(res.info.iter)
                if res.x is not None and np.all(np.isfinite(res.x)):
                    return np.asarray(res.x, dtype=float)
            except Exception as exc:  # pragma: no cover
                self.last_status = "osqp_failed:%s" % exc
        self.last_status = "admm_fallback"
        return self.fallback.solve(H, g, A, l, u, z0=z0)


# --------------------------------------------------------------------------
# unified LTV-MPC
# --------------------------------------------------------------------------
class LTVMPC:
    def __init__(self, prm: ModelParams, cfg):
        self.prm = prm
        self.cfg = cfg
        m = cfg.mpc
        self.dts = np.concatenate([np.full(int(seg.n), float(seg.dt)) for seg in m.horizon])
        self.N = int(len(self.dts))
        self.wheel_on = bool(prm.wheel_on)
        self.nx, self.nu = 5, 2
        self.Q = np.diag([float(m.q_theta), float(m.q_q), 0.0, 0.0, float(m.w_h)])
        self.R = np.diag([float(m.r_delta), (float(m.r_w) if self.wheel_on else 0.0)])
        self.s_delta = float(m.s_delta)
        self.s_w = float(m.s_w)
        self.M_man = float(m.M_man)
        self.k_theta = float(getattr(m, "k_theta", 16.0))
        self.k_q = float(getattr(m, "k_q", 8.0))
        self.n_sqp = int(m.n_sqp)
        self.qp = QPSolver(max_iter=int(getattr(m, "qp_max_iter", 20000)))
        x_scale = np.array([0.01, 0.01, 0.01, 1.0, 10.0])
        u_scale = np.array([0.01, 1.0])
        self.scales = np.concatenate([np.tile(x_scale, self.N), np.tile(u_scale, self.N)])
        self.u_prev = np.zeros(2)
        self.x_seq = None
        self.u_seq = None
        self.z_warm = None
        self.eta = 0.0
        self.gamma_min = 0.0
        self.last_solve_time = 0.0
        self.last_viol = 0.0

    # ---- index helpers --------------------------------------------------
    def _xi(self, k):
        return 5 * (k - 1)

    def _ui(self, k):
        return 5 * self.N + 2 * k

    # ---- authority margin (diagnostic) ----------------------------------
    def _margin(self, x, p, theta_ref, q_ref, qdot_ref):
        prm = self.prm
        Q = aero_Q(p, prm)
        V = max(p[1], 1.0)
        alpha = x[0] - p[2]
        Cm_rest = (prm.Cm0 + prm.Cm_alpha * alpha
                   + prm.Cm_q * x[1] * prm.cbar / (2.0 * V))
        Ma_max = Q * prm.cm_scale * (Cm_rest + prm.Cm_delta * prm.delta_max)
        Ma_min = Q * prm.cm_scale * (Cm_rest - prm.Cm_delta * prm.delta_max)
        M_av = wheel_avail(x[4], prm) if self.wheel_on else 0.0
        qddot_ref = self.k_theta * (theta_ref - x[0]) + self.k_q * (q_ref - x[1]) + qdot_ref
        M_req = prm.Iy * qddot_ref - p[3]
        return (Ma_max + M_av - M_req), (M_req - Ma_min + M_av), M_req

    # ---- initial guess --------------------------------------------------
    def _initial_rollout(self, x0, p_seq, u_seq):
        from legacy.model import plant_step
        xs = [np.array(x0, dtype=float)]
        x = np.array(x0, dtype=float)
        for u, p, dt in zip(u_seq, p_seq, self.dts):
            n = max(1, int(np.ceil(dt / 0.01)))
            for _ in range(n):
                x, _ = plant_step(x, u, p, dt / n, self.prm, substeps=1)
            xs.append(x.copy())
        return np.array(xs)

    # ---- QP assembly ----------------------------------------------------
    def _build_qp(self, x0, x_seq, u_seq, p_seq, t_seq, traj, Pterm):
        N, nx, nu = self.N, self.nx, self.nu
        nz = 7 * N
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
            xt = np.array([traj.theta_ref(t), traj.q_ref(t), 0.0, 0.0, 0.0])
            qi = self._xi(k)
            for i in range(nx):
                if self.Q[i, i] > 0.0:
                    H[qi + i, qi + i] += 2.0 * self.Q[i, i] * dt
                    g[qi + i] += -2.0 * self.Q[i, i] * dt * xt[i]
        for k in range(N):
            dt = self.dts[k]
            ui = self._ui(k)
            for j in range(nu):
                H[ui + j, ui + j] += 2.0 * self.R[j, j] * dt
        for k in range(1, N):
            H[self._ui(k), self._ui(k)] += 2.0 * self.s_delta
            H[self._ui(k), self._ui(k - 1)] += -2.0 * self.s_delta
            H[self._ui(k - 1), self._ui(k - 1)] += 2.0 * self.s_delta
            if self.wheel_on:
                H[self._ui(k) + 1, self._ui(k) + 1] += 2.0 * self.s_w
                H[self._ui(k) + 1, self._ui(k - 1) + 1] += -2.0 * self.s_w
                H[self._ui(k - 1) + 1, self._ui(k - 1) + 1] += 2.0 * self.s_w
        H[self._ui(0), self._ui(0)] += 2.0 * self.s_delta
        g[self._ui(0)] += -2.0 * self.s_delta * self.u_prev[0]
        if self.wheel_on:
            H[self._ui(0) + 1, self._ui(0) + 1] += 2.0 * self.s_w
            g[self._ui(0) + 1] += -2.0 * self.s_w * self.u_prev[1]
        tN = t_seq[N]
        xNref = np.array([traj.theta_ref(tN), traj.q_ref(tN), 0.0, 0.0, 0.0])
        Pi = self._xi(N)
        for i in range(nx):
            for j in range(nx):
                H[Pi + i, Pi + j] += 2.0 * Pterm[i, j]
            g[Pi + i] += -2.0 * (Pterm @ xNref)[i]

        prm = self.prm
        dmax, ddot = prm.delta_max, prm.delta_dot_max
        hmax = prm.h_max

        # ---- equality dynamics ----
        for k in range(N):
            Ad, Bd, d0 = discretize(x_seq[k], u_seq[k], p_seq[k], prm, self.dts[k])
            for i in range(nx):
                coeff = {self._xi(k + 1) + i: 1.0}
                if k == 0:
                    rhs = d0[i] + (Ad @ x0)[i]
                else:
                    for j in range(nx):
                        if Ad[i, j] != 0.0:
                            coeff[self._xi(k) + j] = coeff.get(self._xi(k) + j, 0.0) - Ad[i, j]
                    rhs = d0[i]
                for j in range(nu):
                    if Bd[i, j] != 0.0:
                        coeff[self._ui(k) + j] = coeff.get(self._ui(k) + j, 0.0) - Bd[i, j]
                add(coeff, rhs, rhs)

        # ---- state constraints ----
        for k in range(1, N + 1):
            add({self._xi(k) + 2: 1.0}, -dmax, dmax)          # delta
            add({self._xi(k) + 4: 1.0}, -hmax, hmax)          # h_w
            if self.wheel_on:
                h_ref = x_seq[k][4]
                m0 = wheel_avail(h_ref, prm)
                if prm.P_max * prm.Jw / max(abs(h_ref), 1e-6) < prm.M_stall and abs(h_ref) > 1e-3:
                    m1 = -prm.P_max * prm.Jw * np.sign(h_ref) / (h_ref * h_ref)
                else:
                    m1 = 0.0
                ub = m0 - m1 * h_ref
                add({self._xi(k) + 3: 1.0, self._xi(k) + 4: -m1}, -np.inf, ub)
                add({self._xi(k) + 3: -1.0, self._xi(k) + 4: -m1}, -np.inf, ub)
            if not self.wheel_on:
                add({self._xi(k) + 3: 1.0}, 0.0, 0.0)
                add({self._xi(k) + 4: 1.0}, 0.0, 0.0)

        # ---- input constraints ----
        for k in range(N):
            ui = self._ui(k)
            if k == 0:
                base = x0[2]
                add({ui: 1.0}, -np.inf, ddot * prm.tau_s + base)
                add({ui: -1.0}, -np.inf, ddot * prm.tau_s - base)
            else:
                add({ui: 1.0, self._xi(k) + 2: -1.0}, -np.inf, ddot * prm.tau_s)
                add({ui: -1.0, self._xi(k) + 2: 1.0}, -np.inf, ddot * prm.tau_s)
            add({ui: 1.0}, -dmax, dmax)
            if self.wheel_on:
                add({ui + 1: 1.0}, -prm.M_cmd_max, prm.M_cmd_max)
                if k == 0:
                    add({ui + 1: 1.0}, -np.inf, prm.dM_cmd_max * self.dts[0] + self.u_prev[1])
                    add({ui + 1: -1.0}, -np.inf, prm.dM_cmd_max * self.dts[0] - self.u_prev[1])
                else:
                    lim = prm.dM_cmd_max * self.dts[k]
                    add({ui + 1: 1.0, self._ui(k - 1) + 1: -1.0}, -np.inf, lim)
                    add({ui + 1: -1.0, self._ui(k - 1) + 1: 1.0}, -np.inf, lim)
            else:
                add({ui + 1: 1.0}, 0.0, 0.0)

        A = np.zeros((len(rows), nz))
        for r, coeff in enumerate(rows):
            for c, val in coeff.items():
                A[r, c] = val
        return H, g, A, np.array(lbs), np.array(ubs)

    def _terminal_cost(self, x0, u0, p0):
        try:
            Ad, Bd, _ = discretize(x0, u0, p0, self.prm, float(self.dts[0]))
            Qt = self.Q + 1e-4 * np.eye(5)
            Rt = self.R + 1e-6 * np.eye(2)
            P = solve_discrete_are(Ad, Bd, Qt, Rt)
            return 0.5 * (P + P.T)
        except Exception:
            return self.Q.copy()

    def _pack(self, u_seq, x_seq):
        N = self.N
        z = np.zeros(7 * N)
        z[:5 * N] = x_seq[1:].reshape(-1)
        z[5 * N:7 * N] = u_seq.reshape(-1)
        return z

    def _clamp_prediction(self, z):
        """Keep the SQP linearisation point physically plausible."""
        prm = self.prm
        N = self.N
        X = z[:5 * N].reshape(N, 5).copy()
        U = z[5 * N:7 * N].reshape(N, 2).copy()
        X[:, 0] = np.clip(X[:, 0], -np.pi, np.pi)
        X[:, 1] = np.clip(X[:, 1], -20.0, 20.0)
        X[:, 2] = np.clip(X[:, 2], -prm.delta_max, prm.delta_max)
        X[:, 3] = np.clip(X[:, 3], -2.0 * prm.M_cmd_max, 2.0 * prm.M_cmd_max)
        X[:, 4] = np.clip(X[:, 4], -prm.h_max, prm.h_max)
        U[:, 0] = np.clip(U[:, 0], -prm.delta_max, prm.delta_max)
        U[:, 1] = np.clip(U[:, 1], -prm.M_cmd_max, prm.M_cmd_max)
        return np.concatenate([X.reshape(-1), U.reshape(-1)])

    def _pd_command(self, x0, t, traj):
        err = traj.theta_ref(t) - x0[0]
        rate = traj.q_ref(t) - x0[1]
        return np.array([np.clip(2.0 * err + 0.5 * rate,
                                 -self.prm.delta_max, self.prm.delta_max), 0.0])

    def _shift_warm(self, z):
        if z is None:
            return None
        N = self.N
        zz = np.zeros(7 * N)
        X = z[:5 * N].reshape(N, 5)
        U = z[5 * N:7 * N].reshape(N, 2)
        zz[:5 * N] = np.vstack([X[1:], X[-1:]]).reshape(-1)
        zz[5 * N:7 * N] = np.vstack([U[1:], U[-1:]]).reshape(-1)
        return zz

    def _margin_profile(self, x_seq, p_seq, t_seq, traj):
        gmin = np.inf
        for k in range(self.N):
            t = t_seq[k + 1]
            gp, gm, _ = self._margin(x_seq[k + 1], p_seq[k], traj.theta_ref(t),
                                     traj.q_ref(t), traj.qdot_ref(t))
            gmin = min(gmin, gp, gm)
        return float(gmin)

    # ---- main solve ------------------------------------------------------
    def solve(self, x0, t0, traj, dist_cfg):
        t_start = time.perf_counter()
        x0 = np.array(x0, dtype=float)
        t_seq = np.concatenate([[t0], t0 + np.cumsum(self.dts)])
        p_seq = [traj.p_array(t_seq[k], dist_cfg) for k in range(self.N)]

        if self.u_seq is None:
            u_seq = np.zeros((self.N, 2))
            for k in range(self.N):
                t = t_seq[k]
                err = traj.theta_ref(t) - x0[0]
                rate = traj.q_ref(t) - x0[1]
                u_seq[k, 0] = np.clip(2.0 * err + 0.5 * rate,
                                      -self.prm.delta_max, self.prm.delta_max)
            x_seq = self._initial_rollout(x0, p_seq, u_seq)
        else:
            u_seq = np.vstack([self.u_seq[1:], self.u_seq[-1:]])
            x_seq = np.vstack([self.x_seq[1:], self.x_seq[-1:]])
        u_seq = np.array([limit_commands(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])

        Pterm = self._terminal_cost(x0, u_seq[0], p_seq[0])
        z = self._shift_warm(self.z_warm)
        viol = 0.0
        D = self.scales
        n_iter = self.n_sqp if self.z_warm is not None else max(self.n_sqp, 2)
        failed = False
        for _ in range(max(1, n_iter)):
            H, g, A, lb, ub = self._build_qp(x0, x_seq, u_seq, p_seq, t_seq, traj, Pterm)
            Hs = H * D[:, None] * D[None, :]
            gs = g * D
            As = A * D[None, :]
            z0s = None if z is None else z / D
            z_raw = D * self.qp.solve(Hs, gs, As, lb, ub, z0=z0s)
            Az = A @ z_raw
            dv = np.maximum(lb - Az, Az - ub)
            dv = dv[np.isfinite(dv)]
            viol = float(dv.max()) if dv.size else 0.0
            solved = ("solved" in self.qp.last_status) and np.isfinite(viol) and viol < 1e-4
            if not solved:
                failed = True
                break
            z = self._clamp_prediction(z_raw)
            x_seq = np.vstack([x0, z[:5 * self.N].reshape(self.N, 5)])
            u_seq = z[5 * self.N:7 * self.N].reshape(self.N, 2)
            u_seq = np.array([limit_commands(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])
        if failed:
            # keep the previous (physically valid) command sequence and rebuild the
            # prediction by nonlinear rollout so the next SQP starts from a sane point
            u_seq = np.array([limit_commands(x_seq[k], u_seq[k], self.prm) for k in range(self.N)])
            x_seq = self._initial_rollout(x0, p_seq, u_seq)
            z = self._pack(u_seq, x_seq)

        self.x_seq = x_seq
        self.u_seq = u_seq
        self.z_warm = z
        u0 = limit_commands(x0, u_seq[0], self.prm)
        self.u_prev = u0.copy()
        self.gamma_min = self._margin_profile(x_seq, p_seq, t_seq, traj)
        self.eta = self.gamma_min / max(self.M_man, 1e-9)
        self.last_viol = viol
        self.last_solve_time = time.perf_counter() - t_start
        return u0, dict(eta=self.eta, gamma_min=self.gamma_min, solve_time=self.last_solve_time,
                        x_seq=x_seq, u_seq=u_seq, viol=viol, qp_status=self.qp.last_status,
                        qp_iter=self.qp.last_iter)
