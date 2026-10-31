"""Joint aerodynamic-torque / CMG trajectory optimiser (CasADi + IPOPT).

Multiple-shooting NLP over the 11-state model of nearspace.torque_gap_model:

    decision variables   X[11, k=0..N], U[7, k=0..N-1]
    equality             X[:,k+1] = RK4(X[:,k], U[:,k], tau_d[k], dt_k)
                         || q_k || = 1
                         X[:,0] = x_measured
    inequality           tau_min[k] <= tau_a[k] <= tau_max[k]
                         |nu_i[k]| <= gdot_max
                         |nu_i[k] - nu_i[k-1]| <= gddot_max * dt_k
                         gamma is unwrapped: bounded (+-90 deg) or free in R

    J = J_att + J_reg

The predicted torque-gap modulation term (J_cfg with its auxiliary capability
probe `V`) has been REMOVED: measured over the whole C0-C5 x B1/B2 matrix it gave
no tracking benefit and cost 3x the solver time (see
docs/torque_gap/modulation_removed_report.md).  tau_nom, R+/R- and d are still
computed and logged by the scenario layer as **diagnostics only** -- they no
longer enter the objective, so nothing in this NLP depends on them.

Every cost term is made dimensionless with explicit scales (Weights), and node
costs are weighted by dt_k / horizon so that refining the grid cannot inflate a
term (implementation plan §6.A).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .torque_gap_model import attitude_error_vector


def _casadi():
    """Import CasADi, making the workspace-local solver package loadable.

    On this host casadi is unpacked into ./.pydeps (see requirements-ocp.txt)
    because pip cannot write to site-packages from the experiment sandbox.  The
    Windows loader resolves libcasadi_nlpsol_ipopt.dll's own dependencies through
    PATH, so the package directory must be on PATH (or registered as a DLL
    directory) before `import casadi`, otherwise the IPOPT plugin silently fails
    to load.
    """
    import os
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    deps = root / ".pydeps"
    pkg = deps / "casadi"
    if pkg.is_dir():
        if str(deps) not in sys.path:
            sys.path.insert(0, str(deps))
        d = str(pkg)
        if d.lower() not in os.environ.get("PATH", "").lower():
            os.environ["PATH"] = d + os.pathsep + os.environ.get("PATH", "")
        try:
            os.add_dll_directory(d)
        except (AttributeError, OSError):
            pass
    import casadi as ca
    return ca


@dataclass
class SolverConfig:
    max_iter: int = 400
    tol: float = 1e-6
    print_level: int = 0
    max_cpu_s: float = 4.0
    acceptable: tuple = ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    hessian: str = "exact"              # exact | limited-memory (LM was far worse here)

    @classmethod
    def from_config(cls, cfg):
        s = getattr(getattr(cfg, "torque_gap", None), "solver", None)
        if s is None:
            return cls()

        def g(name, default):
            return getattr(s, name, default)

        return cls(max_iter=int(g("max_iter", 600)), tol=float(g("tol", 1e-6)),
                   print_level=int(g("print_level", 0)),
                   max_cpu_s=float(g("max_cpu_s", 4.0)),
                   hessian=str(g("hessian", "exact")),
                   acceptable=tuple(g("acceptable_status",
                                      ["Solve_Succeeded",
                                       "Solved_To_Acceptable_Level"])))


@dataclass
class OCPResult:
    ok: bool
    status: str
    u0: np.ndarray                 # first applied [tau_a(3), nu(4)]
    X: np.ndarray                  # (11, N+1)
    U: np.ndarray                  # (7, N)
    objective: float
    iter_count: int
    cpu_s: float
    n_vars: int
    n_cons: int
    max_violation: float
    message: str = ""


class TorqueGapOCP:
    """One CasADi graph per (scenario, grid); re-solved every control step."""

    def __init__(self, scn):
        ca = _casadi()
        self.ca = ca
        self.scn = scn
        self.dts = np.asarray(scn.dts, float)
        self.N = int(self.dts.size)
        self.w = scn.w
        self.solver_cfg = SolverConfig.from_config(scn.cfg)
        self.gamma_lim = scn.gamma_lim
        self.tau_rate_limit = scn.tau_rate_limit
        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        ca = self.ca
        scn = self.scn
        N = self.N
        nx, nu = 11, 7
        prm = scn.prm
        J = np.asarray(prm.J, float)
        Jinv = np.asarray(prm.Jinv, float)
        const = prm.cluster_constants()
        h0 = float(const["h0"])
        g_axis = const["g_axis"]
        s0 = const["s0"]
        gxs0 = const["gxs0"]
        gdot_max = float(prm.cmg.gdot_max)
        gddot_max = float(prm.cmg.gddot_max)

        # ---- geometry as CasADi expressions ---------------------------
        gam = ca.SX.sym("gam", 4)
        A_cols, H = [], ca.SX.zeros(3, 1)
        for i in range(4):
            gi = ca.SX(g_axis[:, i])
            s0i = ca.SX(s0[:, i])
            gxi = ca.SX(gxs0[:, i])
            si = s0i * ca.cos(gam[i]) + gxi * ca.sin(gam[i])
            A_cols.append(h0 * ca.cross(gi, si))
            H = H + h0 * si
        A_sym = ca.horzcat(*A_cols)
        self._A_of = ca.Function("A_of", [gam], [A_sym])
        self._H_of = ca.Function("H_of", [gam], [H])

        def quat_mul_sym(a, b):
            aw, ax, ay, az = a[0], a[1], a[2], a[3]
            bw, bx, by, bz = b[0], b[1], b[2], b[3]
            return ca.vertcat(aw * bw - ax * bx - ay * by - az * bz,
                              aw * bx + ax * bw + ay * bz - az * by,
                              aw * by - ax * bz + ay * bw + az * bx,
                              aw * bz + ax * by - ay * bx + az * bw)

        def quat_conj_sym(a):
            return ca.vertcat(a[0], -a[1], -a[2], -a[3])

        x = ca.SX.sym("x", nx)
        u = ca.SX.sym("u", nu)
        td = ca.SX.sym("td", 3)
        q, w, g = x[0:4], x[4:7], x[7:11]
        tau_a, nu_c = u[0:3], u[3:7]
        A_x = ca.horzcat(*[h0 * ca.cross(ca.SX(g_axis[:, i]),
                                        ca.SX(s0[:, i]) * ca.cos(g[i])
                                        + ca.SX(gxs0[:, i]) * ca.sin(g[i]))
                           for i in range(4)])
        H_x = ca.SX.zeros(3, 1)
        for i in range(4):
            H_x = H_x + h0 * (ca.SX(s0[:, i]) * ca.cos(g[i])
                              + ca.SX(gxs0[:, i]) * ca.sin(g[i]))
        f = ca.vertcat(
            0.5 * quat_mul_sym(q, ca.vertcat(0.0, w)),
            ca.mtimes(Jinv, tau_a - ca.mtimes(A_x, nu_c) + td
                      - ca.cross(w, ca.mtimes(J, w) + H_x)),
            nu_c)
        self._f_of = ca.Function("f_tg", [x, u, td], [f])

        def rk4(X, U, TD, dt):
            k1 = self._f_of(X, U, TD)
            k2 = self._f_of(X + 0.5 * dt * k1, U, TD)
            k3 = self._f_of(X + 0.5 * dt * k2, U, TD)
            k4 = self._f_of(X + dt * k3, U, TD)
            Xn = X + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            qn = Xn[0:4] / ca.norm_2(Xn[0:4])
            return ca.vertcat(qn, Xn[4:11])

        # ---- decision vector ------------------------------------------
        nx_tot = nx * (N + 1)
        nu_tot = nu * N
        nz = nx_tot + nu_tot
        Z = ca.SX.sym("Z", nz)
        # CasADi reshape is column-major: Z[s*n + i] is entry i of node s.  The
        # numpy side must therefore reshape with order="F" (see _warm_start/solve).
        X = ca.reshape(Z[0:nx_tot], nx, N + 1)
        U = ca.reshape(Z[nx_tot:nx_tot + nu_tot], nu, N)

        # ---- parameters ------------------------------------------------
        p_qref = ca.SX.sym("qref", 4, N + 1)
        p_wref = ca.SX.sym("wref", 3, N)
        p_lo = ca.SX.sym("lo", 3, N)
        p_hi = ca.SX.sym("hi", 3, N)
        p_td = ca.SX.sym("td", 3, N)
        p_nuprev = ca.SX.sym("nuprev", 4)
        p_tauprev = ca.SX.sym("tauprev", 3)
        p_xinit = ca.SX.sym("xinit", nx)
        P = ca.vertcat(ca.vec(p_qref), ca.vec(p_wref), ca.vec(p_lo), ca.vec(p_hi),
                       ca.vec(p_td), p_nuprev, p_tauprev, p_xinit)
        self._p_shapes = dict(qref=(4, N + 1), wref=(3, N), lo=(3, N), hi=(3, N),
                              td=(3, N), nuprev=(4,), tauprev=(3,),
                              xinit=(nx,))

        W = self.w
        H_horizon = float(np.sum(self.dts))
        J = 0.0
        J_terms = []          # diagnostics only: the additive cost terms
        g_terms, g_lb, g_ub = [], [], []

        # initial state is pinned to the measurement
        g_terms.append(X[:, 0] - p_xinit)
        g_lb.extend([0.0] * nx)
        g_ub.extend([0.0] * nx)

        for k in range(N):
            dt = float(self.dts[k])
            wk = dt / H_horizon
            qk, wk_om = X[0:4, k], X[4:7, k]
            qref = p_qref[:, k]
            qe = quat_mul_sym(quat_conj_sym(qref), qk)
            eR = 2.0 * qe[1:4]
            dw = wk_om - p_wref[:, k]
            t_att = wk * (W.w_att * ca.dot(eR, eR) / W.e_scale ** 2
                          + W.w_rate * ca.dot(dw, dw) / W.omega_scale ** 2)
            J += t_att
            J_terms.append(("att[%d]" % k, t_att))

            # dynamics (multiple shooting defect)
            Xn = rk4(X[:, k], U[:, k], p_td[:, k], dt)
            g_terms.append(X[:, k + 1] - Xn)
            g_lb.extend([0.0] * nx)
            g_ub.extend([0.0] * nx)

            # gimbal-rate increment (previous *executed* rate included)
            prev = p_nuprev if k == 0 else U[3:7, k - 1]
            dnu = U[3:7, k] - prev
            for i in range(4):
                g_terms.append(dnu[i])
                g_lb.append(-gddot_max * dt)
                g_ub.append(gddot_max * dt)
            # optional aerodynamic torque rate limit (ablation)
            if self.tau_rate_limit is not None:
                dtau = U[0:3, k] - (p_tauprev if k == 0 else U[0:3, k - 1])
                for i in range(3):
                    g_terms.append(dtau[i])
                    g_lb.append(-float(self.tau_rate_limit) * dt)
                    g_ub.append(float(self.tau_rate_limit) * dt)

            # regularisation
            nu_k = U[3:7, k]
            dv = nu_k - prev
            t_reg = wk * (W.r_nu * ca.dot(nu_k, nu_k) / W.nu_scale ** 2
                          + W.r_dnu * ca.dot(dv, dv) / W.nu_scale ** 2)
            dtau_k = U[0:3, k] - (p_tauprev if k == 0 else U[0:3, k - 1])
            t_reg += wk * W.r_dtau * ca.dot(dtau_k, dtau_k) / W.d_scale ** 2
            J += t_reg
            J_terms.append(("reg[%d]" % k, t_reg))

        # terminal attitude weight
        qN = X[0:4, N]
        qeN = quat_mul_sym(quat_conj_sym(p_qref[:, N]), qN)
        eRN = 2.0 * qeN[1:4]
        t_term = W.q_terminal * ca.dot(eRN, eRN) / W.e_scale ** 2
        J += t_term
        J_terms.append(("terminal", t_term))

        # unit-norm quaternion equality at every node
        for k in range(N + 1):
            g_terms.append(ca.dot(X[0:4, k], X[0:4, k]))
            g_lb.append(1.0)
            g_ub.append(1.0)

        g = ca.vertcat(*g_terms)
        # kept for diagnostics: individual constraint blocks can be evaluated
        self._Z, self._P, self._g_terms = Z, P, g_terms
        self._J_terms = J_terms

        # ---- bounds ----------------------------------------------------
        lbx = np.full(nz, -np.inf)
        ubx = np.full(nz, np.inf)
        if scn.gamma_limit_mode != "none":
            # gimbal travel is a *position* limit only in the bounded model;
            # with gamma_limit: none the state stays a free, unwrapped angle and
            # only the rate / acceleration limits below remain
            for k in range(N + 1):
                base = k * nx
                lbx[base + 7:base + 11] = self.gamma_lim[0]
                ubx[base + 7:base + 11] = self.gamma_lim[1]
        u_off = nx_tot
        nu_lo, nu_hi = (-gdot_max, gdot_max) if scn.cmg_enabled else (0.0, 0.0)
        for k in range(N):
            base = u_off + k * nu
            lbx[base:base + 3] = np.nan          # placeholder, set per solve
            ubx[base:base + 3] = np.nan
            lbx[base + 3:base + 7] = nu_lo
            ubx[base + 3:base + 7] = nu_hi
        v_off = nx_tot + nu_tot
        vmax = gdot_max if scn.cmg_enabled else 0.0
        lbx[v_off:] = -vmax
        ubx[v_off:] = vmax
        self._tau_idx = (u_off, nu, N)
        self._nz, self._nx_tot, self._nu_tot = nz, nx_tot, nu_tot
        self._lbx_template = lbx
        self._ubx_template = ubx
        self._g_lb = np.asarray(g_lb, float)
        self._g_ub = np.asarray(g_ub, float)

        opts = dict(ipopt=dict(
            max_iter=self.solver_cfg.max_iter,
            tol=self.solver_cfg.tol,
            print_level=self.solver_cfg.print_level,
            linear_solver="mumps",
            mu_strategy="adaptive",
            warm_start_init_point="yes",
            warm_start_bound_push=1e-8,
            warm_start_mult_bound_push=1e-8,
            bound_relax_factor=1e-10,
            nlp_scaling_method="gradient-based",
            expect_infeasible_problem="no",
            hessian_approximation=("limited-memory"
                                   if self.solver_cfg.hessian == "limited-memory"
                                   else "exact"),
            sb="yes",
        ), print_time=False)
        nlp = dict(x=Z, p=P, f=J, g=g)
        self.solver = ca.nlpsol("torque_gap_ocp", "ipopt", nlp, opts)
        self.n_vars = int(nz)
        self.n_cons = int(g.shape[0])

    # ------------------------------------------------------------------
    def _param_vector(self, plan, x_init, nu_prev, tau_prev):
        N = self.N
        cond, box = plan["cond"], plan["box"]
        vals = [np.asarray(cond["q_ref"], float).T,                     # (4, N+1)
                np.asarray(cond["omega_ref"], float)[:N].T,             # (3, N)
                np.asarray(box["tau_min"], float)[:N].T,                # (3, N)
                np.asarray(box["tau_max"], float)[:N].T,                # (3, N)
                np.asarray(cond["tau_d"], float)[:N].T,                 # (3, N)
                np.asarray(nu_prev, float).reshape(4),
                np.asarray(tau_prev, float).reshape(3),
                np.asarray(x_init, float).reshape(11)]
        # ca.vec() in the graph stacks matrices column-major, so the numpy side
        # must flatten with order="F" too (same convention as X/U/V above).
        return np.concatenate([np.asarray(v, float).reshape(-1, order="F") for v in vals])

    def _bounds(self, plan):
        lbx = self._lbx_template.copy()
        ubx = self._ubx_template.copy()
        u_off, nu, N = self._tau_idx
        lo, hi = plan["box"]["tau_min"], plan["box"]["tau_max"]
        for k in range(N):
            base = u_off + k * nu
            lbx[base:base + 3] = lo[k]
            ubx[base:base + 3] = hi[k]
        return lbx, ubx

    def _warm_start(self, plan, x_init, nu_prev, tau_prev, warm):
        z = np.zeros(self._nz)
        X = z[0:self._nx_tot].reshape(11, self.N + 1, order="F")
        U = z[self._nx_tot:self._nx_tot + self._nu_tot].reshape(7, self.N, order="F")
        if warm is None:
            # cold start: propagate the measured state with the feed-forward torque
            # guess so that the initial guess satisfies the shooting defects
            from .torque_gap_model import rk4_step_tg
            X[:, 0] = np.asarray(x_init, float)
            tau_guess = np.asarray(plan["mod"]["tau_nom"], float).T   # (3, N+1)
            U[0:3, :] = tau_guess[:, :self.N]
            U[3:7, :] = np.asarray(nu_prev, float)[:, None]
            for k in range(self.N):
                X[:, k + 1] = rk4_step_tg(X[:, k], U[:, k], plan["cond"]["tau_d"][k],
                                          float(self.dts[k]), self.scn.prm,
                                          self.scn.cluster)
                if not self.scn.gamma_unbounded:
                    X[7:11, k + 1] = np.clip(X[7:11, k + 1], self.gamma_lim[0],
                                             self.gamma_lim[1])
        else:
            # shift the previous solution by one node and repeat the last action
            Xw, Uw = warm
            X[:, :-1] = Xw[:, 1:]
            X[:, -1] = Xw[:, -1]
            U[:, :-1] = Uw[:, 1:] if Uw.shape[1] > 1 else Uw[:, :1]
            U[:, -1] = Uw[:, -1]
        # honour the bounds (they change every step)
        x0 = np.asarray(x_init, float)
        X[:, 0] = x0
        return z

    # ------------------------------------------------------------------
    def solve(self, plan, x_init, nu_prev, tau_prev, warm=None):
        import time
        t0 = time.perf_counter()
        p = self._param_vector(plan, x_init, nu_prev, tau_prev)
        lbx, ubx = self._bounds(plan)
        z0 = self._warm_start(plan, x_init, nu_prev, tau_prev, warm)
        z0 = np.clip(z0, lbx, ubx)
        status, message, ok = "not_called", "", False
        zsol = z0
        obj = np.inf
        it = 0
        gsol = None
        try:
            sol = self.solver(x0=z0, p=p, lbx=lbx, ubx=ubx, lbg=self._g_lb,
                              ubg=self._g_ub)
            stats = self.solver.stats()
            status = stats.get("return_status", "unknown")
            it = int(stats.get("iter_count", 0))
            message = stats.get("nlp_solver", "")
            zsol = np.asarray(sol["x"]).ravel()
            gsol = np.asarray(sol["g"]).ravel()
            obj = float(sol["f"])
            ok = status in self.solver_cfg.acceptable
        except Exception as exc:                      # noqa: BLE001
            status = "exception"
            message = "%s: %s" % (type(exc).__name__, exc)
            ok = False

        X = zsol[0:self._nx_tot].reshape(11, self.N + 1, order="F")
        U = zsol[self._nx_tot:self._nx_tot + self._nu_tot].reshape(7, self.N, order="F")
        viol = self._max_violation(lbx, ubx, zsol, gsol)
        cpu = time.perf_counter() - t0
        return OCPResult(ok=ok, status=status, u0=U[:, 0].copy(), X=X.copy(),
                         U=U.copy(), objective=obj, iter_count=it,
                         cpu_s=cpu, n_vars=self.n_vars, n_cons=self.n_cons,
                         max_violation=viol, message=message)

    def _max_violation(self, lbx, ubx, Z, G):
        """Largest bound / constraint violation of a (possibly failed) solution."""
        v = float(np.max(np.abs(Z - np.clip(Z, lbx, ubx)))) if Z.size else 0.0
        if G is not None:
            vc = max(float(np.max(G - self._g_ub)), float(np.max(self._g_lb - G)))
            v = max(v, vc)
        return float(v)
