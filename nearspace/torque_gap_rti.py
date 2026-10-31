"""Real-time-iteration (RTI) MPC backend for the torque-gap problem, using HPIPM.

The IPOPT backend of nearspace.torque_gap_ocp solves the *whole* nonlinear
multiple-shooting NLP to convergence every control period: 35-145 iterations of a
407-variable problem whose Lagrangian Hessian is completely dense (165 649
non-zeros), i.e. 0.24-1.39 s per control step against a 0.2 s period.

This module replaces that with the standard real-time-iteration scheme:

    per control step
        1. roll the measured state forward with the previous (shifted) control
           sequence                     -> reference trajectory (x̄_k, ū_k)
        2. linearise the RK4 map        -> A_k, B_k,  b_k = 0
        3. build the Gauss-Newton QP: Hessian from the attitude/rate error
           Jacobian, gradient from the current error, control box constraints
        4. solve the *structured* OCP-QP with HPIPM (condensing + Riccati,
           BLASFEO kernels)             -> δx, δu
        5. apply ū_0 + δu_0, update the reference for the next step

`qp_iters > 1` turns the same code into a Gauss-Newton SQP that is iterated to
convergence, which is how the QP is validated against the IPOPT solution.

Differences from the IPOPT NLP, all documented and measured:
  * Gauss-Newton Hessian (no second derivatives of the dynamics);
  * the gimbal-acceleration coupling between consecutive stages is represented by
    a regularisation around the reference increment; the hard limit is still
    enforced by the executor (`ClosedLoop` clips nu to nu_prev ± gddot·dt);
  * the QP is solved to tolerance, not iterated until the *nonlinear* KKT
    conditions hold (that is what `qp_iters` is for).
"""
from __future__ import annotations

import time

import numpy as np

from . import hpipm_bindings
from .torque_gap_model import attitude_error_vector, quat_mul, quat_normalize
from .torque_gap_ocp import OCPResult, _casadi


def _quat_mul_np(a, b):
    return quat_mul(np.asarray(a, float), np.asarray(b, float))


class TorqueGapRTI:
    """Same interface as TorqueGapOCP: `.N`, `.n_vars`, `.n_cons`, `.solve(...)`."""

    def __init__(self, scn, qp_iters=1, qp_iter_max=40, qp_tol=1e-6, mode="balance",
                 line_search=True, step_tol=1e-4,
                 state_tr_gamma=0.0, state_tr_omega=0.0, state_tr_sigma=False,
                 nu_scale_sigma=False, qp_warm_start=False, cost_scale=False,
                 gamma_damp=0.0, qp_iters_hi=0):
        self.ca = ca = _casadi()
        self.scn = scn
        self.dts = np.asarray(scn.dts, float)
        self.N = int(self.dts.size)
        self.w = scn.w
        self.nx = int(scn.nx)
        self.nu = int(scn.nu)
        self.qp_iters = int(qp_iters)
        self.use_line_search = bool(line_search)
        self.step_tol = float(step_tol)
        self.cmg_present = bool(scn.cmg_present)
        self.gamma_lim = scn.gamma_lim
        self.horizon = float(np.sum(self.dts))
        # --- linearisation-stability options (all off = historical behaviour) ---
        self.state_tr_gamma = float(state_tr_gamma)     # |dx_gamma| per stage [rad]
        self.state_tr_omega = float(state_tr_omega)     # |dx_omega| per stage [rad/s]
        self.state_tr_sigma = bool(state_tr_sigma)      # tighten the gamma TR near singular
        self.nu_scale_sigma = bool(nu_scale_sigma)      # tighten |dnu| near singular
        self.qp_warm_start = bool(qp_warm_start)
        self.cost_scale = bool(cost_scale)              # normalise the QP cost to O(1)
        self.gamma_damp = float(gamma_damp)             # damping on |dgamma|^2
        self.qp_iters_hi = int(qp_iters_hi)             # extra QPs when near singular
        self.cluster = scn.cluster if self.cmg_present else None
        if self.cmg_present:
            self.sigma_floor = float(scn.prm.cmg.sigma_floor)
            self.sigma_home = float(scn.prm.cmg.home_sigma_min)
        self.last_sigma_min = np.inf
        self._build_model()
        self._build_qp(qp_iter_max, qp_tol, mode)
        self.n_vars = self.nx * (self.N + 1) + self.nu * self.N
        self.n_cons = self.nx * (self.N + 1) + self.nu * self.N

    # ------------------------------------------------------------------
    @classmethod
    def from_config(cls, scn, **over):
        """Build from `torque_gap.rti.*` (all keys optional)."""
        r = getattr(getattr(scn.cfg, "torque_gap", None), "rti", None)

        def g(name, default):
            v = over.get(name, None)
            if v is None:
                v = getattr(r, name, default) if r is not None else default
            return v

        return cls(scn,
                   qp_iters=int(g("qp_iters", 12)),
                   qp_iter_max=int(g("qp_iter_max", 40)),
                   qp_tol=float(g("qp_tol", 1e-6)),
                   mode=str(g("mode", "balance")),
                   line_search=bool(g("line_search", True)),
                   step_tol=float(g("step_tol", 1e-4)),
                   state_tr_gamma=float(g("state_tr_gamma", 0.0)),
                   state_tr_omega=float(g("state_tr_omega", 0.0)),
                   state_tr_sigma=bool(g("state_tr_sigma", False)),
                   nu_scale_sigma=bool(g("nu_scale_sigma", False)),
                   qp_warm_start=bool(g("qp_warm_start", False)),
                   cost_scale=bool(g("cost_scale", False)),
                   gamma_damp=float(g("gamma_damp", 0.0)),
                   qp_iters_hi=int(g("qp_iters_hi", 0)))

    def options(self):
        """The stability/accuracy knobs, for the run metadata."""
        return dict(qp_iters=int(self.qp_iters), qp_iters_hi=int(self.qp_iters_hi),
                    line_search=bool(self.use_line_search),
                    step_tol=float(self.step_tol),
                    state_tr_gamma=float(self.state_tr_gamma),
                    state_tr_omega=float(self.state_tr_omega),
                    state_tr_sigma=bool(self.state_tr_sigma),
                    nu_scale_sigma=bool(self.nu_scale_sigma),
                    qp_warm_start=bool(self.qp_warm_start),
                    cost_scale=bool(self.cost_scale),
                    gamma_damp=float(self.gamma_damp),
                    sigma_min_last=float(self.last_sigma_min))

    # ------------------------------------------------------------------
    def _build_model(self):
        """CasADi functions: RK4 map and its Jacobians, attitude error Jacobian."""
        ca = self.ca
        scn = self.scn
        nx, nu_in = self.nx, self.nu
        prm = scn.prm
        J = np.asarray(prm.J, float)
        Jinv = np.asarray(prm.Jinv, float)

        def quat_mul_sym(a, b):
            aw, ax, ay, az = a[0], a[1], a[2], a[3]
            bw, bx, by, bz = b[0], b[1], b[2], b[3]
            return ca.vertcat(aw * bw - ax * bx - ay * by - az * bz,
                              aw * bx + ax * bw + ay * bz - az * by,
                              aw * by - ax * bz + ay * bw + az * bx,
                              aw * bz + ax * by - ay * bx + az * bw)

        x = ca.SX.sym("x", nx)
        u = ca.SX.sym("u", nu_in)
        td = ca.SX.sym("td", 3)
        dt = ca.SX.sym("dt")
        q, w = x[0:4], x[4:7]
        tau_a = u[0:3]
        f0 = 0.5 * quat_mul_sym(q, ca.vertcat(0.0, w))
        wdot = ca.mtimes(Jinv, tau_a + td - ca.cross(w, ca.mtimes(J, w)))
        if self.cmg_present:
            h0 = float(scn.prm.cmg.h0)
            cl = scn.prm.cluster_constants()
            g = x[7:11]
            nu_c = u[3:7]
            A_x = ca.horzcat(*[h0 * ca.cross(ca.SX(cl["g_axis"][:, i]),
                                             ca.SX(cl["s0"][:, i]) * ca.cos(g[i])
                                             + ca.SX(cl["gxs0"][:, i]) * ca.sin(g[i]))
                               for i in range(4)])
            H_x = ca.SX.zeros(3, 1)
            for i in range(4):
                H_x = H_x + h0 * (ca.SX(cl["s0"][:, i]) * ca.cos(g[i])
                                  + ca.SX(cl["gxs0"][:, i]) * ca.sin(g[i]))
            wdot = ca.mtimes(Jinv, tau_a - ca.mtimes(A_x, nu_c) + td
                             - ca.cross(w, ca.mtimes(J, w) + H_x))
            f = ca.vertcat(f0, wdot, nu_c)
        else:
            f = ca.vertcat(f0, wdot)
        fd = ca.Function("f_rti", [x, u, td], [f])

        def rk4(Xk, Uk, TD, h):
            k1 = fd(Xk, Uk, TD)
            k2 = fd(Xk + 0.5 * h * k1, Uk, TD)
            k3 = fd(Xk + 0.5 * h * k2, Uk, TD)
            k4 = fd(Xk + h * k3, Uk, TD)
            Xn = Xk + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            qn = Xn[0:4] / ca.norm_2(Xn[0:4])
            return ca.vertcat(qn, Xn[4:nx])

        Xn = rk4(x, u, td, dt)
        F = ca.Function("F_rti", [x, u, td, dt], [Xn])
        Aof = ca.Function("A_rti", [x, u, td, dt], [ca.jacobian(Xn, x)])
        Bof = ca.Function("B_rti", [x, u, td, dt], [ca.jacobian(Xn, u)])
        self._F, self._A, self._B = F, Aof, Bof

        qref = ca.SX.sym("qref", 4)
        qe = quat_mul_sym(ca.vertcat(qref[0], -qref[1], -qref[2], -qref[3]), x[0:4])
        e = 2.0 * qe[1:4]
        eof = ca.Function("e_rti", [qref, x], [e, ca.jacobian(e, x)])
        self._eof = eof
        self._Ew = np.zeros((3, nx))
        self._Ew[:, 4:7] = np.eye(3)          # selector for the body-rate block

    # ------------------------------------------------------------------
    def _build_qp(self, iter_max, tol, mode, reg_prim=1e-10):
        cls = hpipm_bindings.classes()
        N, nx, nu_in = self.N, self.nx, self.nu
        dim = cls["dim"](N)
        dim.set("nx", nx, 0, N)
        dim.set("nu", nu_in, 0, N - 1)
        dim.set("nbx", nx, 0)                 # dx_0 = 0 (measured state is fixed)
        dim.set("nbxe", nx, 0)                # ... as an equality constraint
        dim.set("nbu", nu_in, 0, N - 1)       # control box at every stage

        # state trust region on the deviation (keeps the linearisation inside the
        # region where the gimbal geometry A(gamma) is still the one we linearised)
        rows, bounds = [], []
        if self.cmg_present:
            if self.state_tr_gamma > 0.0:
                rows.extend([7, 8, 9, 10])
                bounds.extend([self.state_tr_gamma] * 4)
            if self.state_tr_omega > 0.0:
                rows.extend([4, 5, 6])
                bounds.extend([self.state_tr_omega] * 3)
        self._tr_rows = np.asarray(rows, int)
        self._tr_bound_vec = np.asarray(bounds, float)
        self._tr_active = self._tr_rows.size > 0
        self._Jx_tr = np.zeros((self._tr_rows.size, nx))
        for i, r in enumerate(self._tr_rows):
            self._Jx_tr[i, r] = 1.0
        if self._tr_active:
            for k in range(1, N + 1):
                dim.set("nbx", int(self._tr_rows.size), k)
                dim.set("nbxe", 0, k)
        else:
            for k in range(1, N + 1):
                dim.set("nbx", 0, k)

        arg = cls["arg"](dim, mode)
        arg.set("iter_max", int(iter_max))
        arg.set("tol_stat", float(tol))
        arg.set("tol_eq", float(tol))
        arg.set("tol_ineq", float(tol))
        arg.set("tol_comp", float(tol))
        arg.set("reg_prim", float(reg_prim))
        if self.qp_warm_start:
            # the qp_sol object persists, so this re-uses the previous QP solution
            arg.set("warm_start", 1)
        self._dim = dim
        self._qp = cls["qp"](dim)
        self._sol = cls["sol"](dim)
        self._solver = cls["solver"](dim, arg)
        self._cls = cls
        self._Jx = np.eye(nx)
        self._Ju = np.eye(nu_in)

    # ------------------------------------------------------------------
    def _rollout(self, x0, U, plan):
        X = np.zeros((self.nx, self.N + 1))
        X[:, 0] = x0
        tau_d = np.asarray(plan["cond"]["tau_d"], float)
        for k in range(self.N):
            X[:, k + 1] = np.asarray(
                self._F(X[:, k], U[:, k], tau_d[k], self.dts[k])).ravel()
        return X

    def _weights(self):
        w = self.w
        wk = self.dts / self.horizon
        qa = wk * 2.0 * w.w_att / w.e_scale ** 2
        qr = wk * 2.0 * w.w_rate / w.omega_scale ** 2
        qt = 2.0 * w.q_terminal / w.e_scale ** 2
        return qa, qr, qt, wk

    def _sigma_profile(self, X):
        """sigma_min(A(gamma_k)) at every node (only when a sigma-aware option is on)."""
        if not self.cmg_present:
            return None
        if not (self.nu_scale_sigma or self.state_tr_sigma or self.qp_iters_hi > 0):
            return None
        cl = self.cluster
        return np.array([cl.sigma_min(cl.jacobian(X[7:11, k]))
                         for k in range(self.N + 1)])

    def _qp_matrices(self, X, U, plan, sig=None):
        """Gauss-Newton QP data for the current reference trajectory."""
        N, nx, nu_in = self.N, self.nx, self.nu
        W = self.w
        qa, qr, qt, wk = self._weights()
        qref = np.asarray(plan["cond"]["q_ref"], float)
        wref = np.asarray(plan["cond"]["omega_ref"], float)
        lo = np.asarray(plan["box"]["tau_min"], float)
        hi = np.asarray(plan["box"]["tau_max"], float)
        gdot = float(self.scn.prm.cmg.gdot_max)
        gddot = float(self.scn.prm.cmg.gddot_max)
        nu_on = self.cmg_present and bool(self.scn.cmg_enabled)

        Q = np.zeros((N + 1, nx, nx))
        qv = np.zeros((N + 1, nx))
        R = np.zeros((N, nu_in, nu_in))
        rv = np.zeros((N, nu_in))
        lu = np.zeros((N, nu_in))
        uu = np.zeros((N, nu_in))
        for k in range(N + 1):
            e, Je = self._eof(qref[k], X[:, k])
            e = np.asarray(e).ravel()
            Je = np.asarray(Je)
            wgt = qa[k] if k < N else qt
            Q[k] += wgt * (Je.T @ Je)
            qv[k] += wgt * (Je.T @ e)
            if k < N:
                dW = X[4:7, k] - wref[k]
                Q[k] += qr[k] * (self._Ew.T @ self._Ew)
                qv[k] += qr[k] * (self._Ew.T @ dW)
                if self.cmg_present and self.gamma_damp > 0.0:
                    # damping on the gimbal-angle deviation: keeps the plan from
                    # winding the gimbals when the linear model is unreliable
                    Q[k][7:11, 7:11] += self.gamma_damp * wk[k] * np.eye(4)
        for k in range(N):
            if self.cmg_present:
                r_nu = wk[k] * 2.0 * W.r_nu / W.nu_scale ** 2
                r_dnu = wk[k] * 2.0 * W.r_dnu / W.nu_scale ** 2
                R[k][3:7, 3:7] += (r_nu + r_dnu) * np.eye(4)
                rv[k][3:7] += r_nu * U[3:7, k]
                if k > 0:
                    rv[k][3:7] += r_dnu * (U[3:7, k] - U[3:7, k - 1])
            r_dtau = wk[k] * 2.0 * W.r_dtau / W.d_scale ** 2
            R[k][0:3, 0:3] += r_dtau * np.eye(3)
            if k > 0:
                rv[k][0:3] += r_dtau * (U[0:3, k] - U[0:3, k - 1])
            # The gimbal-rate *increment* limit of the NLP couples consecutive
            # stages, which a stage-wise QP cannot express.  It is imposed here as
            # a hard trust region on the deviation of this stage's rate:
            #   |du_nu| <= gddot * dt_k   (optionally scaled down near singularity)
            # and the executor still clips nu to nu_prev +- gddot*dt.  Without this
            # bound the QP happily drives the gimbals to +-gdot for the whole
            # horizon, which is useless because the linear model breaks down.
            span = gddot * float(self.dts[k])
            if self.nu_scale_sigma and sig is not None:
                f = np.clip(sig[k] / max(self.sigma_floor, 1e-9), 0.15, 1.0)
                span = span * float(f)
            if nu_on:
                nu_lo = np.maximum(-gdot - U[3:7, k], -span)
                nu_hi = np.minimum(gdot - U[3:7, k], span)
            else:
                nu_lo = -U[3:7, k]
                nu_hi = -U[3:7, k]
            lu[k] = np.concatenate([lo[k] - U[0:3, k], nu_lo])
            uu[k] = np.concatenate([hi[k] - U[0:3, k], nu_hi])

        if self.cost_scale:
            # The tracking Hessian is ~500 while the control regularisation is
            # ~1e-4; a global cost scale leaves the solution unchanged but makes
            # the interior-point sub-problems far better conditioned.
            diag = max(float(np.max(np.abs(np.diagonal(Q, axis1=1, axis2=2)))),
                       float(np.max(np.abs(np.diagonal(R, axis1=1, axis2=2)))),
                       1e-12)
            Q = Q / diag
            qv = qv / diag
            R = R / diag
            rv = rv / diag
        return Q, qv, R, rv, lu, uu

    def _tr_bounds(self, k, sig):
        """State trust-region bounds of stage k (empty when inactive)."""
        if not self._tr_active:
            return None
        b = self._tr_bound_vec.copy()
        if self.state_tr_sigma and sig is not None and self.cmg_present:
            n_g = 4 if self.state_tr_gamma > 0.0 else 0
            if n_g:
                f = np.clip(sig[k] / max(self.sigma_floor, 1e-9), 0.2, 1.0)
                b[:n_g] = b[:n_g] * float(f)
        return b

    # ------------------------------------------------------------------
    def _iterate(self, X, U, plan, sig=None):
        """One Gauss-Newton SQP iteration: linearise, solve the QP, update."""
        N, nx, nu_in = self.N, self.nx, self.nu
        tau_d = np.asarray(plan["cond"]["tau_d"], float)
        Ak = np.zeros((N, nx, nx))
        Bk = np.zeros((N, nx, nu_in))
        for k in range(N):
            Ak[k] = np.asarray(self._A(X[:, k], U[:, k], tau_d[k], self.dts[k]))
            Bk[k] = np.asarray(self._B(X[:, k], U[:, k], tau_d[k], self.dts[k]))
        Q, qv, R, rv, lu, uu = self._qp_matrices(X, U, plan, sig)

        # NOTE: the wrapper's range form (idx_start, idx_end) writes the *same*
        # array to every stage, so per-stage data must be set one stage at a time.
        qp = self._qp
        b_zero = np.zeros(nx)
        for k in range(N):
            qp.set("A", Ak[k], k)
            qp.set("B", Bk[k], k)
            qp.set("b", b_zero, k)
            qp.set("R", R[k], k)
            qp.set("r", rv[k], k)
            qp.set("Ju", self._Ju, k)
            qp.set("lu", lu[k], k)
            qp.set("uu", uu[k], k)
        for k in range(N + 1):
            qp.set("Q", Q[k], k)
            qp.set("q", qv[k], k)
        qp.set("Jx", self._Jx, 0)
        qp.set("lx", np.zeros(nx), 0)
        qp.set("ux", np.zeros(nx), 0)
        if self._tr_active:
            for k in range(1, N + 1):
                b = self._tr_bounds(k, sig)
                qp.set("Jx", self._Jx_tr, k)
                qp.set("lx", -b, k)
                qp.set("ux", b, k)

        self._solver.solve(qp, self._sol)
        status = int(self._solver.get("status"))
        iters = int(self._solver.get("iter"))
        dX = np.zeros((nx, N + 1))
        dU = np.zeros((nu_in, N))
        for k in range(N):
            dU[:, k] = np.asarray(self._sol.get("u", k)).ravel()
        for k in range(N + 1):
            dX[:, k] = np.asarray(self._sol.get("x", k)).ravel()
        return dX, dU, status, iters

    # ------------------------------------------------------------------
    def _initial_guess(self, plan, x_init, warm):
        """(X, U): the shifted previous plan (or the nominal feed-forward)."""
        N, nu_in = self.N, self.nu
        if warm is not None:
            Xw, Uw = warm
            U = np.empty((nu_in, N))
            U[:, :-1] = Uw[:, 1:]
            U[:, -1] = Uw[:, -1]
        else:
            U = np.zeros((nu_in, N))
            U[0:3, :] = np.asarray(plan["mod"]["tau_nom"], float).T[:, :N]
        X = self._rollout(np.asarray(x_init, float), U, plan)
        return X, U

    # ------------------------------------------------------------------
    def _clip(self, U, plan):
        lo = np.asarray(plan["box"]["tau_min"], float)
        hi = np.asarray(plan["box"]["tau_max"], float)
        U = U.copy()
        U[0:3, :] = np.clip(U[0:3, :], lo[:self.N].T, hi[:self.N].T)
        if self.cmg_present:
            gdot = float(self.scn.prm.cmg.gdot_max)
            U[3:7, :] = np.clip(U[3:7, :], -gdot, gdot)
        return U

    def _line_search(self, x_init, U, dU, plan, obj0):
        """Backtracking line search on the *nonlinear* objective.

        A single Gauss-Newton step is only locally valid: on this problem the raw
        QP step drives the aerodynamic torque to the box over the whole horizon,
        because the linear model promises a tracking gain the nonlinear plant
        cannot deliver.  Scaling the step and keeping the best rollout keeps the
        reference trajectory meaningful -- and that reference is what the next
        warm start is built from.
        """
        best = (obj0, U, None, 0.0)
        for alpha in (1.0, 0.5, 0.25, 0.125):
            Un = self._clip(U + alpha * dU, plan)
            Xn = self._rollout(x_init, Un, plan)
            on = self._objective(Xn, Un, plan)
            if on < best[0]:
                best = (on, Un, Xn, alpha)
        return best

    def solve(self, plan, x_init, nu_prev=None, tau_prev=None, warm=None):
        t0 = time.perf_counter()
        x_init = np.asarray(x_init, float).ravel()
        X, U = self._initial_guess(plan, x_init, warm)
        lo = np.asarray(plan["box"]["tau_min"], float)
        hi = np.asarray(plan["box"]["tau_max"], float)
        gdot = float(self.scn.prm.cmg.gdot_max)
        U[0:3, :] = np.clip(U[0:3, :], lo[:self.N].T, hi[:self.N].T)
        if self.cmg_present:
            U[3:7, :] = np.clip(U[3:7, :], -gdot, gdot)

        status, iters_total = 0, 0
        obj = self._objective(X, U, plan)
        sig = self._sigma_profile(X)
        if sig is not None:
            self.last_sigma_min = float(np.min(sig))
        # adaptive effort: spend more QPs only where the linearisation is fragile
        iters_eff = max(self.qp_iters, 1)
        if self.qp_iters_hi > 0 and sig is not None and \
                self.last_sigma_min < self.sigma_floor:
            iters_eff = max(iters_eff, self.qp_iters_hi)
        for _ in range(iters_eff):
            dX, dU, status, it = self._iterate(X, U, plan, sig)
            iters_total += it
            if status != 0:
                break
            step = float(np.max(np.abs(dU))) if dU.size else 0.0
            if self.use_line_search:
                obj, Un, Xn, alpha = self._line_search(x_init, U, dU, plan, obj)
                if Xn is None:                 # no descent found: keep the plan
                    break
                U, X = Un, Xn
            else:
                U = self._clip(U + dU, plan)
                X = self._rollout(x_init, U, plan)
                obj = self._objective(X, U, plan)
            # early exit: the QP step is below actuator resolution, further
            # Gauss-Newton iterations would only burn time
            if step < self.step_tol:
                break
        u0 = U[:, 0].copy()
        viol = float(np.max(np.abs(u0[0:3] - np.clip(u0[0:3], lo[0], hi[0]))))
        cpu = time.perf_counter() - t0
        return OCPResult(ok=(status == 0),
                         status="rti_ok" if status == 0 else "rti_status_%d" % status,
                         u0=u0, X=X, U=U, objective=float(self._objective(X, U, plan)),
                         iter_count=iters_total, cpu_s=cpu, n_vars=self.n_vars,
                         n_cons=self.n_cons, max_violation=viol, message="hpipm")

    # ------------------------------------------------------------------
    def _objective(self, X, U, plan):
        """The NLP stage cost at (X, U) — used only for reporting."""
        W = self.w
        qa, qr, qt, wk = self._weights()
        qref = np.asarray(plan["cond"]["q_ref"], float)
        wref = np.asarray(plan["cond"]["omega_ref"], float)
        J = 0.0
        for k in range(self.N):
            e = attitude_error_vector(qref[k], X[0:4, k])
            dw = X[4:7, k] - wref[k]
            J += 0.5 * qa[k] * float(e @ e) + 0.5 * qr[k] * float(dw @ dw)
        eN = attitude_error_vector(qref[self.N], X[0:4, self.N])
        J += 0.5 * qt * float(eN @ eN)
        return J
