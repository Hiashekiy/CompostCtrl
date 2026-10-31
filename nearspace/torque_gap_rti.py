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
                 line_search=True, step_tol=1e-4):
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
        self._build_model()
        self._build_qp(qp_iter_max, qp_tol, mode)
        self.n_vars = self.nx * (self.N + 1) + self.nu * self.N
        self.n_cons = self.nx * (self.N + 1) + self.nu * self.N

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
        arg = cls["arg"](dim, mode)
        arg.set("iter_max", int(iter_max))
        arg.set("tol_stat", float(tol))
        arg.set("tol_eq", float(tol))
        arg.set("tol_ineq", float(tol))
        arg.set("tol_comp", float(tol))
        arg.set("reg_prim", float(reg_prim))
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

    def _qp_matrices(self, X, U, plan):
        """Gauss-Newton QP data for the current reference trajectory."""
        N, nx, nu_in = self.N, self.nx, self.nu
        W = self.w
        qa, qr, qt, wk = self._weights()
        qref = np.asarray(plan["cond"]["q_ref"], float)
        wref = np.asarray(plan["cond"]["omega_ref"], float)
        tau_d = np.asarray(plan["cond"]["tau_d"], float)
        lo = np.asarray(plan["box"]["tau_min"], float)
        hi = np.asarray(plan["box"]["tau_max"], float)
        gdot = float(self.scn.prm.cmg.gdot_max)
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
            #   |du_nu| <= gddot * dt_k
            # and the executor still clips nu to nu_prev +- gddot*dt.  Without this
            # bound the QP happily drives the gimbals to +-gdot for the whole
            # horizon, which is useless because the linear model breaks down.
            span = float(self.scn.prm.cmg.gddot_max) * float(self.dts[k])
            if nu_on:
                nu_lo = np.maximum(-gdot - U[3:7, k], -span)
                nu_hi = np.minimum(gdot - U[3:7, k], span)
            else:
                nu_lo = -U[3:7, k]
                nu_hi = -U[3:7, k]
            lu[k] = np.concatenate([lo[k] - U[0:3, k], nu_lo])
            uu[k] = np.concatenate([hi[k] - U[0:3, k], nu_hi])
        return Q, qv, R, rv, lu, uu

    # ------------------------------------------------------------------
    def _iterate(self, X, U, plan):
        """One Gauss-Newton SQP iteration: linearise, solve the QP, update."""
        N, nx, nu_in = self.N, self.nx, self.nu
        tau_d = np.asarray(plan["cond"]["tau_d"], float)
        Ak = np.zeros((N, nx, nx))
        Bk = np.zeros((N, nx, nu_in))
        for k in range(N):
            Ak[k] = np.asarray(self._A(X[:, k], U[:, k], tau_d[k], self.dts[k]))
            Bk[k] = np.asarray(self._B(X[:, k], U[:, k], tau_d[k], self.dts[k]))
        Q, qv, R, rv, lu, uu = self._qp_matrices(X, U, plan)

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
        for _ in range(max(self.qp_iters, 1)):
            dX, dU, status, it = self._iterate(X, U, plan)
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
