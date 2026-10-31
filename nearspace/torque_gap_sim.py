"""Closed-loop runner for the torque-gap joint optimisation (P0 / P1).

Two plants:

P0 (``plant: P0``)  the P0 virtual-torque truth of nearspace.torque_gap_model:
                    11 states with a CMG, 7 states without one.
P1 (``plant: P1``)  the legacy 18-state truth of nearspace.model6c (real servo and
                    gimbal-rate loops).  The optimiser torque command tau_a is
                    converted to a bounded surface deflection by
                    nearspace.torque_gap_authority.allocate_deflection and the
                    achieved aerodynamic moment is logged separately.  With
                    ``cmg_present: false`` the same 18-state truth runs with
                    ``VehicleCMGParams.cmg_on == False``, i.e. no CMG torque, no
                    rotor momentum and a structurally frozen gimbal.  P0 and P1
                    results are never mixed.

Log time alignment (implementation guide §9.1)
    Row k covers the control interval [t, t + dt_ctrl).  ``t`` is the control
    period start (the time at which the plan / command / solve time belong);
    ``t_next = t + dt_ctrl`` is the time of the state that the row reports.  The
    attitude reference, the reference body rate, the attitude/rate errors and all
    error statistics therefore use ``t_next``, never ``t``.

CMG torque convention (implementation guide §9.2)
    ``cmg_[xyz]`` is the *control-interval average* of the achieved CMG torque,
    built from the integrator sub-steps (P0: -A(gamma_i) nu held over each
    sub-step; P1: trapezoidal average of -A(gamma) gdot over the gimbal-loop
    sub-steps).  ``cmg_end_[xyz]`` is the end-of-interval instantaneous value.
    The two are never mixed.  In the A group (no CMG) all CMG columns are empty
    ("not applicable") -- no fabricated zero curves.

Every control step logs the solver status, iterations, wall clock and the maximum
constraint violation.  A step whose NLP did not converge, violated a constraint
or exceeded the wall-clock budget is *not* used: the runner applies the fallback
(previous verified action, re-checked against the current actuator limits, or a
bounded stabilising controller) and records it.
"""
from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
from dataclasses import dataclass

import numpy as np

from .cmg import CMGCluster
from .model6 import quat_to_euler, wrap_pi
from .model6c import VehicleCMGParams, aero_moment6c, plant_step6c
from .torque_gap_authority import allocate_deflection, natural_moment_v, surface_matrix
from .torque_gap_model import (attitude_error_vector, rk4_step_tg,
                               rk4_step_tg_nocmg, split_state, split_state_nocmg)
from .torque_gap_modulation import capability_residual, nominal_demand

# Columns that exist for every run.
COMMON_COLUMNS = (
    ["t", "t_next", "h_km", "rho", "Q", "step"]
    + ["q%d" % i for i in range(4)] + ["qref%d" % i for i in range(4)]
    + ["wx", "wy", "wz", "wrefx", "wrefy", "wrefz"]
    + ["att_err_deg", "rate_err"]
    + ["err_x", "err_y", "err_z", "e_roll_deg", "e_pitch_deg", "e_yaw_deg"]
    + ["tauc_cmd_x", "tauc_cmd_y", "tauc_cmd_z"]           # commanded tau_a
    + ["tau_act_x", "tau_act_y", "tau_act_z"]              # achieved tau_a
    + ["tau_nom_x", "tau_nom_y", "tau_nom_z"]
    + ["Rp_x", "Rp_y", "Rp_z", "Rm_x", "Rm_y", "Rm_z", "d_x", "d_y", "d_z"]
    + ["lo_x", "lo_y", "lo_z", "hi_x", "hi_y", "hi_z"]
    + ["tau_bound_used"]
    + ["cpu_s", "iters", "viol", "fallback", "status"]
)

# Columns that only exist when the vehicle carries a CMG (B group).  They are
# written as empty cells in the A group: "not applicable", not a zero curve.
CMG_COLUMNS = (
    ["g1", "g2", "g3", "g4", "nu1", "nu2", "nu3", "nu4",
     "nuc1", "nuc2", "nuc3", "nuc4"]
    + ["Hx", "Hy", "Hz", "H_norm", "sigma_min"]
    + ["cmg_x", "cmg_y", "cmg_z"]                          # interval-average -A nu
    + ["cmg_end_x", "cmg_end_y", "cmg_end_z"]              # end-of-interval -A nu
    + ["E_gap", "gap_residual", "nu_sat"]
)

# Columns that only exist for the P1 plant (real servo / deflection allocation).
P1_COLUMNS = (
    ["de_cmd", "da_cmd", "dr_cmd", "de_act", "da_act", "dr_act",
     "alloc_res_x", "alloc_res_y", "alloc_res_z"]
)

LOG_COLUMNS = tuple(COMMON_COLUMNS + CMG_COLUMNS + P1_COLUMNS)


@dataclass
class RunMeta:
    git_sha: str
    git_branch: str
    python: str
    packages: dict
    config_path: str
    case: str
    mode: str
    plant: str
    seed: int


def _git(field):
    try:
        return subprocess.run(["git", field], cwd=os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), capture_output=True, text=True,
            check=False).stdout.strip()
    except Exception:                                       # noqa: BLE001
        return ""


def _packages():
    import importlib
    out = {}
    for name in ("numpy", "scipy", "matplotlib", "yaml", "casadi"):
        try:
            out[name] = importlib.import_module(name).__version__
        except Exception:                                   # noqa: BLE001
            out[name] = "absent"
    return out


def gimbal_interval_average(cluster: CMGCluster, gamma, gdot, nu_cmd, dt,
                            substeps=8, tau_g=0.03):
    """Sub-step average of the achieved CMG torque over one control interval.

    The gimbal loop (gamma_dot = gdot, gdot_dot = (nu_cmd - gdot)/tau_g) is
    independent of the rest of the 18-state plant, so it is integrated here with
    the same RK4 sub-step structure as the truth model.  Returns
    (cmg_average, gamma_end, gdot_end) with cmg_average the sub-step mean of
    -A(gamma) gdot (mid-point rule, i.e. integral / dt).
    """
    gamma = np.asarray(gamma, float).copy()
    gdot = np.asarray(gdot, float).copy()
    n = max(int(substeps), 1)
    h = dt / float(n)
    acc = np.zeros(3)
    for _ in range(n):
        g0, d0 = gamma.copy(), gdot.copy()
        k1g, k1d = d0, (nu_cmd - d0) / tau_g
        k2g, k2d = d0 + 0.5 * h * k1d, (nu_cmd - (d0 + 0.5 * h * k1d)) / tau_g
        k3g, k3d = d0 + 0.5 * h * k2d, (nu_cmd - (d0 + 0.5 * h * k2d)) / tau_g
        k4g, k4d = d0 + h * k3d, (nu_cmd - (d0 + h * k3d)) / tau_g
        gamma = g0 + (h / 6.0) * (k1g + 2.0 * k2g + 2.0 * k3g + k4g)
        gdot = d0 + (h / 6.0) * (k1d + 2.0 * k2d + 2.0 * k3d + k4d)
        acc = acc + h * (-cluster.torque(0.5 * (g0 + gamma), 0.5 * (d0 + gdot)))
    return acc / dt, gamma, gdot


class ClosedLoop:
    def __init__(self, scn, ocp=None, plant="P0"):
        from .torque_gap_ocp import SolverConfig, TorqueGapOCP
        self.scn = scn
        self.cmg_present = bool(scn.cmg_present)
        self.ocp = ocp or TorqueGapOCP(scn)
        self.plant = str(plant).upper()
        self.cfg = scn.cfg
        self.solver_cfg = SolverConfig.from_config(scn.cfg)
        self.substeps = int(getattr(scn.cfg.torque_gap, "plant_substeps", 8))
        self.dt = float(scn.dt_ctrl)
        self.n_steps = int(round((scn.t_end - scn.t_start) / self.dt))
        self.vp = VehicleCMGParams.from_config(
            scn.cfg, cmg_on=self.cmg_present)        # P1 truth: CMG on/off
        self.cluster6c = CMGCluster(self.vp.cmg) if self.cmg_present else None
        self.gimbal_mismatch_max = 0.0
        self.gamma_clipped_steps = 0
        self.delta = np.zeros(3)
        self.rows = []
        self.log = {}

    # ---- columns ---------------------------------------------------
    @property
    def columns(self):
        cols = list(COMMON_COLUMNS) + list(CMG_COLUMNS)
        if self.plant != "P0":
            cols += list(P1_COLUMNS)
        return cols

    @property
    def log_conventions(self):
        return dict(
            state_time="t_next = t + dt_ctrl (attitude/rate reference and all errors)",
            control_time="t (plan, command, solve time, limits)",
            cmg_torque="cmg_* = control-interval average of the achieved -A(gamma) nu "
                       "(integrator sub-steps); cmg_end_* = end-of-interval instantaneous",
            cmg_columns_in_A_group="empty cells: not applicable (no CMG)",
        )

    # ---- plant-specific execution ---------------------------------
    def _tau_d(self, t):
        return np.asarray(self.scn.traj.disturbance(t, self.cfg, self.scn.veh), float)

    def _step_P0(self, x, tau_a, nu, t):
        """One control interval of the P0 truth plant.

        Returns (x_next, tau_achieved, nu_used, cmg_avg, cmg_end).
        """
        n = self.substeps
        h = self.dt / float(n)
        if self.cmg_present:
            gamma_lim = self.scn.gamma_lim
            acc = np.zeros(3)
            for i in range(n):
                td = self._tau_d(t + (i + 0.5) * h)
                g0 = x[7:11].copy()
                u = np.concatenate([tau_a, nu])
                x = rk4_step_tg(x, u, td, h, self.scn.prm, self.scn.cluster)
                if not self.scn.gamma_unbounded:
                    x[7:11] = np.clip(x[7:11], gamma_lim[0], gamma_lim[1])
                acc = acc + h * (-self.scn.cluster.torque(0.5 * (g0 + x[7:11]), nu))
            cmg_avg = acc / self.dt
            cmg_end = -self.scn.cluster.torque(x[7:11], nu)
            return x, tau_a.copy(), nu.copy(), cmg_avg, cmg_end
        for i in range(n):
            td = self._tau_d(t + (i + 0.5) * h)
            x = rk4_step_tg_nocmg(x, tau_a, td, h, self.scn.prm)
        return x, tau_a.copy(), np.zeros(4), None, None

    def _step_P1(self, x18, tau_a_des, nu_cmd, t):
        """18-state truth: torque -> bounded deflection, gimbal rate commanded."""
        q, w = x18[0:4], x18[4:7]
        rho = float(self.scn.traj.rho(t))
        V_ned = np.asarray(self.scn.traj.V_ned(t), float)
        V = float(np.linalg.norm(V_ned))
        Q = 0.5 * rho * V ** 2
        p = np.concatenate([[rho], V_ned, self._tau_d(t)])
        xprobe = np.concatenate([q, w, np.zeros(3), x18[10:14], np.zeros(4)])
        from .model6 import flow_angles
        alpha, beta, _ = flow_angles(xprobe, p)
        nat = natural_moment_v(Q, alpha, beta, w, V, self.scn.veh)
        M = surface_matrix(Q, self.scn.veh)
        span = np.asarray(self.scn.veh.delta_dot_max, float) * self.dt
        delta_cmd, res = allocate_deflection(np.asarray(tau_a_des) - nat, M,
                                             self.scn.veh, delta_prev=self.delta,
                                             ddelta_max=span)
        self.delta = delta_cmd
        u18 = np.concatenate([delta_cmd,
                              nu_cmd if self.cmg_present else np.zeros(4)])
        cmg_avg = cmg_end = None
        if self.cmg_present:
            cmg_avg, gam_aux, gdot_aux = gimbal_interval_average(
                self.cluster6c, x18[10:14], x18[14:18], u18[3:7], self.dt,
                substeps=self.substeps, tau_g=float(self.vp.cmg.tau_g))
        x18n, u_act = plant_step6c(x18, u18, p, self.dt, self.vp, self.cluster6c,
                                   substeps=self.substeps)
        # physical gimbal travel stops: the legacy 18-state truth has no travel
        # limit, so it is applied here exactly as in the P0 plant.  Without this
        # the truth gimbal angle runs away (|gamma| >> pi/2), the measured state
        # leaves the optimiser's gimbal box and every subsequent NLP is infeasible.
        if not self.scn.gamma_unbounded:
            lim = self.scn.gamma_lim
            x18n[10:14] = np.clip(x18n[10:14], lim[0], lim[1])
        if self.cmg_present:
            self.gimbal_mismatch_max = max(
                self.gimbal_mismatch_max,
                float(np.max(np.abs(x18n[10:18] - np.concatenate([gam_aux, gdot_aux])))))
            cmg_end = -self.cluster6c.torque(x18n[10:14], x18n[14:18])
        tau_act = aero_moment6c(x18n, p, self.vp)
        return (x18n, tau_act, np.asarray(u_act[3:7], float).copy(),
                np.asarray(delta_cmd, float), res, cmg_avg, cmg_end)

    def _init_plant(self):
        x = self.scn.initial_state()
        if self.plant == "P0":
            return x
        x18 = np.zeros(18)
        x18[0:4] = x[0:4]
        x18[4:7] = x[4:7]
        if self.cmg_present:
            x18[10:14] = x[7:11]
        return x18

    def _state_opt(self, x):
        """The state vector handed to the optimiser (7 without CMG, 11 with)."""
        if self.plant == "P0":
            return x
        if self.cmg_present:
            return np.concatenate([x[0:4], x[4:7], x[10:14]])
        return np.concatenate([x[0:4], x[4:7]])

    # ---- fallback --------------------------------------------------
    def _fallback(self, plan, x, nu_prev, tau_prev):
        """Bounded stabilising action when the NLP result cannot be used.

        1. the first action of the previous feasible plan, re-checked against the
           current aerodynamic box and (B group) the gimbal rate / acceleration
           and travel limits;
        2. if that is not admissible, a proportional attitude stabiliser; the B
           group additionally fills the predicted torque gap by a bounded
           least-squares gimbal rate.  The A group has no gimbal rate to fill
           anything with.
        """
        lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
        gdot = float(self.scn.prm.cmg.gdot_max)
        gddot = float(self.scn.prm.cmg.gddot_max)
        if self._last_action is not None:
            tau_a = np.clip(self._last_action[0:3], lo, hi)
            if not self.cmg_present:
                return tau_a, "previous"
            nu = np.clip(self._last_action[3:7], -gdot, gdot)
            nu = np.clip(nu, nu_prev - gddot * self.dt, nu_prev + gddot * self.dt)
            return np.concatenate([tau_a, nu]), "previous"
        if self.cmg_present:
            q, w, _ = split_state(x)
        else:
            q, w = split_state_nocmg(x)
        qref = plan["cond"]["q_ref"][0]
        wref = plan["cond"]["omega_ref"][0]
        eR = attitude_error_vector(qref, q)
        wdot = 2.0 / self.dt * (0.05 * eR - (w - wref))          # simple PD law
        tau_ff = nominal_demand(self.scn.traj, plan["cond"]["times"][0], self.scn.prm,
                                plan["mod"]["H_nom"], plan["cond"]["tau_d"][0])
        tau_a = np.clip(tau_ff + self.scn.prm.J @ wdot, lo, hi)
        if not self.cmg_present:
            return tau_a, "stabiliser"
        d = plan["mod"]["d"][0]
        nu = np.zeros(4)
        if np.any(np.abs(d) > 0):
            A = self.scn.cluster.jacobian(x[7:11])
            from scipy.optimize import lsq_linear
            span = gddot * self.dt
            nu = lsq_linear(A, -d, bounds=(np.maximum(-gdot, nu_prev - span),
                                           np.minimum(gdot, nu_prev + span)),
                            method="bvls").x
        return np.concatenate([tau_a, np.asarray(nu, float)]), "stabiliser"

    # ---- main loop -------------------------------------------------
    def run(self):
        scn, ocp = self.scn, self.ocp
        x = self._init_plant()
        nu_prev = np.zeros(4)
        tau_prev = np.zeros(3)
        self._last_action = None
        warm = None
        gdot = float(scn.prm.cmg.gdot_max)
        gddot = float(scn.prm.cmg.gddot_max)
        gamma_lim = scn.gamma_lim
        self.rows = []
        self.gamma_clipped_steps = 0
        for step in range(self.n_steps):
            t = scn.t_start + step * self.dt
            t_next = t + self.dt
            x_opt = self._state_opt(x)
            if self.cmg_present:
                # the optimiser's gimbal box must contain the measured configuration
                g_clip = (np.clip(x_opt[7:11], gamma_lim[0], gamma_lim[1])
                          if not scn.gamma_unbounded else x_opt[7:11])
                if np.any(g_clip != x_opt[7:11]):
                    self.gamma_clipped_steps += 1
                    x_opt = x_opt.copy()
                    x_opt[7:11] = g_clip
            plan = scn.plan(t, x_opt)
            res = ocp.solve(plan, x_opt, nu_prev, tau_prev, warm)
            fallback = 0
            ok = (res.ok and res.max_violation <= 1e-4
                  and res.cpu_s <= self.solver_cfg.max_cpu_s)
            if ok:
                u_cmd = res.u0.copy()
                status = res.status
                warm = (res.X, res.U)
            else:
                u_cmd, why = self._fallback(plan, x_opt, nu_prev, tau_prev)
                status = "%s|%s" % (res.status, why)
                fallback = 1
            # execution-side actuator limits (independent of the optimiser)
            lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
            tau_a_cmd = np.clip(u_cmd[0:3], lo, hi)
            if self.cmg_present and scn.cmg_enabled:
                nu_cmd = np.clip(u_cmd[3:7], -gdot, gdot)
                nu_cmd = np.clip(nu_cmd, nu_prev - gddot * self.dt,
                                 nu_prev + gddot * self.dt)
            else:
                nu_cmd = np.zeros(4)          # no CMG (A) / CMG locked

            delta_cmd = np.zeros(3)
            alloc_res = np.zeros(3)
            if self.plant == "P0":
                x, tau_act, nu_act, cmg_avg, cmg_end = self._step_P0(
                    x, tau_a_cmd, nu_cmd, t)
            else:
                (x, tau_act, nu_act, delta_cmd, alloc_res,
                 cmg_avg, cmg_end) = self._step_P1(x, tau_a_cmd, nu_cmd, t)
            self._last_action = np.concatenate([tau_a_cmd, nu_cmd])
            nu_prev = np.asarray(nu_act, float)
            tau_prev = np.asarray(tau_act, float)
            self.rows.append(self._log_row(step, t, t_next, x, plan, res, tau_a_cmd,
                                           tau_act, nu_cmd, nu_act, delta_cmd,
                                           alloc_res, cmg_avg, cmg_end, fallback,
                                           status))
        return self._summary()

    # ---- logging ---------------------------------------------------
    def _log_row(self, step, t, t_next, x, plan, res, tau_a_cmd, tau_act, nu_cmd,
                 nu_act, delta_cmd, alloc_res, cmg_avg, cmg_end, fallback, status):
        scn = self.scn
        traj = scn.traj
        # state and reference both belong to t_next (implementation guide §9.1)
        if self.cmg_present:
            q, w, gamma = split_state(self._state_opt(x))
        else:
            q, w = split_state_nocmg(self._state_opt(x))
            gamma = None
        qref = traj.q_ref(t_next)
        wref = np.asarray(traj.omega_ref(t_next), float)
        eR = attitude_error_vector(qref, q)
        e_eul = np.array([wrap_pi(a - b) for a, b in
                          zip(quat_to_euler(qref), quat_to_euler(q))])
        lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
        tol = 1e-6 * np.maximum(1.0, np.abs(hi - lo))
        at_bound = np.mean((np.abs(tau_act - lo) <= tol) |
                           (np.abs(tau_act - hi) <= tol))
        row = dict(
            t=t, t_next=t_next, h_km=traj.h(t_next) / 1000.0,
            rho=plan["cond"]["rho"][0], Q=plan["box"]["Q"][0], step=step,
            **{"q%d" % i: q[i] for i in range(4)},
            **{"qref%d" % i: qref[i] for i in range(4)},
            wx=w[0], wy=w[1], wz=w[2], wrefx=wref[0], wrefy=wref[1], wrefz=wref[2],
            att_err_deg=np.rad2deg(np.linalg.norm(eR)),
            rate_err=float(np.linalg.norm(w - wref)),
            err_x=eR[0], err_y=eR[1], err_z=eR[2],
            e_roll_deg=np.rad2deg(e_eul[0]), e_pitch_deg=np.rad2deg(e_eul[1]),
            e_yaw_deg=np.rad2deg(e_eul[2]),
            tauc_cmd_x=tau_a_cmd[0], tauc_cmd_y=tau_a_cmd[1], tauc_cmd_z=tau_a_cmd[2],
            tau_act_x=tau_act[0], tau_act_y=tau_act[1], tau_act_z=tau_act[2],
            **{("tau_nom_%s" % ax): plan["mod"]["tau_nom"][0][i]
               for i, ax in enumerate("xyz")},
            **{("Rp_%s" % ax): plan["mod"]["R_plus"][0][i] for i, ax in enumerate("xyz")},
            **{("Rm_%s" % ax): plan["mod"]["R_minus"][0][i] for i, ax in enumerate("xyz")},
            **{("d_%s" % ax): plan["mod"]["d"][0][i] for i, ax in enumerate("xyz")},
            **{("lo_%s" % ax): lo[i] for i, ax in enumerate("xyz")},
            **{("hi_%s" % ax): hi[i] for i, ax in enumerate("xyz")},
            tau_bound_used=float(at_bound),
            cpu_s=res.cpu_s, iters=res.iter_count, viol=res.max_violation,
            fallback=fallback, status=status,
        )
        if self.cmg_present:
            A = scn.cluster.jacobian(gamma)
            H = scn.cluster.momentum(gamma)
            d = plan["mod"]["d"][0]
            E = capability_residual(gamma, d, scn.cluster,
                                    float(scn.prm.cmg.gdot_max))[0]
            end = -scn.cluster.torque(gamma, nu_act)
            row.update(
                **{"g%d" % (i + 1): gamma[i] for i in range(4)},
                **{"nu%d" % (i + 1): nu_act[i] for i in range(4)},
                **{"nuc%d" % (i + 1): nu_cmd[i] for i in range(4)},
                Hx=H[0], Hy=H[1], Hz=H[2], H_norm=float(np.linalg.norm(H)),
                sigma_min=scn.cluster.sigma_min(A),
                **{("cmg_%s" % ax): float(np.asarray(cmg_avg)[i])
                   for i, ax in enumerate("xyz")},
                **{("cmg_end_%s" % ax): float(end[i]) for i, ax in enumerate("xyz")},
                E_gap=E, gap_residual=float(np.linalg.norm(end - d)),
                nu_sat=float(np.mean(np.abs(nu_cmd)
                                     >= 0.999 * float(scn.prm.cmg.gdot_max))))
        if self.plant != "P0":
            dd = np.asarray(delta_cmd, float)
            row.update(
                de_cmd=dd[0], da_cmd=dd[1], dr_cmd=dd[2],
                de_act=x[7], da_act=x[8], dr_act=x[9],
                alloc_res_x=float(alloc_res[0]), alloc_res_y=float(alloc_res[1]),
                alloc_res_z=float(alloc_res[2]))
        return row

    # ---- summary ---------------------------------------------------
    def _summary(self):
        rows = self.rows
        scn = self.scn
        t = np.array([r["t_next"] for r in rows])
        err = np.array([r["att_err_deg"] for r in rows])

        def arr(key):
            return np.array([float(r[key]) for r in rows])

        def win_stats(mask):
            if not mask.any():
                return None
            e = err[mask]
            tau = np.vstack([np.array([r["tau_act_x"], r["tau_act_y"], r["tau_act_z"]])
                             for i, r in enumerate(rows) if mask[i]])
            out = dict(
                rmse_deg=float(np.sqrt(np.mean(e ** 2))), max_err_deg=float(np.max(e)),
                rate_rmse=float(np.sqrt(np.mean(arr("rate_err")[mask] ** 2))),
                err_axis_deg=dict(
                    x=float(np.rad2deg(np.sqrt(np.mean(arr("err_x")[mask] ** 2)))),
                    y=float(np.rad2deg(np.sqrt(np.mean(arr("err_y")[mask] ** 2)))),
                    z=float(np.rad2deg(np.sqrt(np.mean(arr("err_z")[mask] ** 2))))),
                tau_rms=[float(v) for v in np.sqrt(np.mean(tau ** 2, axis=0))],
                tau_peak=[float(v) for v in np.max(np.abs(tau), axis=0)],
                tau_bound_used=float(np.mean(arr("tau_bound_used")[mask])),
                fallback_ratio=float(np.mean(arr("fallback")[mask])),
                cpu_mean=float(np.mean(arr("cpu_s")[mask])),
            )
            if self.cmg_present:
                cmg = np.vstack([np.array([r["cmg_x"], r["cmg_y"], r["cmg_z"]])
                                 for i, r in enumerate(rows) if mask[i]])
                out["cmg_tau_rms"] = [float(v) for v in np.sqrt(np.mean(cmg ** 2, axis=0))]
                out["cmg_tau_peak"] = [float(v) for v in np.max(np.abs(cmg), axis=0)]
                out["H_norm_max"] = float(np.max(arr("H_norm")[mask]))
                out["sigma_min_min"] = float(np.min(arr("sigma_min")[mask]))
                out["nu_sat_ratio"] = float(np.mean(arr("nu_sat")[mask]))
            if self.plant != "P0":
                dact = np.vstack([np.array([r["de_act"], r["da_act"], r["dr_act"]])
                                  for i, r in enumerate(rows) if mask[i]])
                out["delta_rms_deg"] = [float(v) for v in
                                        np.rad2deg(np.sqrt(np.mean(dact ** 2, axis=0)))]
                out["delta_peak_deg"] = [float(v) for v in
                                         np.rad2deg(np.max(np.abs(dact), axis=0))]
                # achieved deflection-rate saturation: |d(delta)/dt| at the limit
                rate = np.abs(np.diff(dact, axis=0)) / self.dt
                lim = np.asarray(scn.veh.delta_dot_max, float)[None, :]
                if rate.size:
                    out["delta_rate_sat_ratio"] = float(np.mean(rate >= 0.999 * lim))
                out["delta_pos_sat_ratio"] = float(np.mean(
                    np.abs(dact) >= 0.999 * np.asarray(scn.veh.delta_max, float)[None, :]))
                diff = np.vstack([np.array([r["tauc_cmd_x"] - r["tau_act_x"],
                                            r["tauc_cmd_y"] - r["tau_act_y"],
                                            r["tauc_cmd_z"] - r["tau_act_z"]])
                                  for i, r in enumerate(rows) if mask[i]])
                out["tau_cmd_vs_act_rms"] = [float(v) for v in
                                             np.sqrt(np.mean(diff ** 2, axis=0))]
            return out

        maneuvers = [m for m in scn.traj.maneuvers if m.enabled]
        windows = {}
        for i, m in enumerate(maneuvers):
            windows["man%d" % i] = (m.t0, m.t_end, m.axis, float(np.rad2deg(m.delta)))
        if maneuvers:
            windows["combined"] = (min(m.t0 for m in maneuvers),
                                   max(m.t_end for m in maneuvers), "multi", None)
            windows["pre"] = (min(m.t0 for m in maneuvers) - 12.0,
                              min(m.t0 for m in maneuvers), "pre", None)
            windows["post"] = (max(m.t_end for m in maneuvers) + 1e-9,
                               float(t[-1]) + 1e-9, "post", None)
        windows["all"] = (float(t[0]) - 1e-9, float(t[-1]) + 1e-9, "all", None)
        stats = {}
        masks = {}
        for name, (t0, t1, _ax, _dv) in windows.items():
            mask = (t >= t0) & (t < t1)
            masks[name] = mask
            s = win_stats(mask)
            if s is not None:
                stats[name] = s

        out = dict(
            plant=self.plant, modulation="removed",
            cmg_present=self.cmg_present, cmg_enabled=bool(scn.cmg_enabled),
            nx_model=int(scn.nx), nu_model=int(scn.nu),
            nlp_vars=int(self.ocp.n_vars), nlp_constraints=int(self.ocp.n_cons),
            t_start=scn.t_start, t_end=scn.t_end, dt_ctrl=self.dt,
            n_steps=len(rows), seed=int(getattr(scn, "seed", 0)),
            windows={k: dict(t0=v[0], t1=v[1], axis=v[2], delta_deg=v[3])
                     for k, v in windows.items()},
            stats=stats,
            # convenience mirrors of the aggregate numbers
            rmse_deg={k: v["rmse_deg"] for k, v in stats.items()},
            max_err_deg={k: v["max_err_deg"] for k, v in stats.items()},
            cpu_s=dict(mean=float(np.mean(arr("cpu_s"))),
                       p95=float(np.percentile(arr("cpu_s"), 95)),
                       max=float(np.max(arr("cpu_s"))),
                       total=float(np.sum(arr("cpu_s")))),
            realtime_dt_ctrl=self.dt,
            realtime_met=bool(np.percentile(arr("cpu_s"), 95) <= self.dt),
            fallback_steps=int(arr("fallback").sum()),
            fallback_ratio=float(np.mean(arr("fallback"))),
            iterations=dict(mean=float(np.mean(arr("iters"))),
                            max=int(np.max(arr("iters")))),
            max_violation=float(np.max(arr("viol"))),
            status_counts={s: int(sum(1 for r in rows if r["status"] == s))
                           for s in sorted({r["status"] for r in rows})},
            gamma_clipped_steps=int(self.gamma_clipped_steps),
        )
        # settle time and CMG migration, per manoeuvre window
        for i, m in enumerate(maneuvers):
            name = "man%d" % i
            if name not in stats:
                continue
            sel = t >= m.t0
            bad = np.flatnonzero((err > 1.0) & sel & (t <= m.t_end + 30.0))
            stats[name]["settle_time_s"] = (float(t[bad[-1]] - m.t0 + self.dt)
                                            if bad.size else 0.0)
        if maneuvers:
            i0 = int(np.argmax(t >= maneuvers[0].t0))
            g0 = np.array([rows[0].get("g%d" % (i + 1), np.nan) for i in range(4)])
            if self.cmg_present and not np.any(np.isnan(g0)):
                dev = np.array([max(abs(rows[i]["g%d" % (k + 1)] - g0[k])
                                    for k in range(4)) for i in range(len(rows))])
                prem = t < maneuvers[0].t0
                nus = np.array([max(abs(r["nu%d" % (i + 1)]) for i in range(4))
                                for r in rows])
                lead = np.flatnonzero((dev > 0.05) & prem)       # > ~3 deg
                out["cmg_migration"] = dict(
                    gamma_at_start=[float(rows[i0]["g%d" % (i + 1)]) for i in range(4)],
                    H_at_start=[float(rows[i0]["Hx"]), float(rows[i0]["Hy"]),
                                float(rows[i0]["Hz"])],
                    H_norm_at_start=float(rows[i0]["H_norm"]),
                    pre_gimbal_travel_rad=float(np.sum(nus[prem]) * self.dt),
                    gamma_dev_max_deg=float(np.rad2deg(np.max(dev))),
                    migration_onset_t=float(t[lead[0]]) if lead.size else None,
                    migration_lead_s=(float(maneuvers[0].t0 - t[lead[0]])
                                      if lead.size else 0.0))
        if self.plant != "P0":
            out["p1_gimbal_mismatch_max"] = float(self.gimbal_mismatch_max)
        return out


# --------------------------------------------------------------------------
# output helpers
# --------------------------------------------------------------------------
def write_timeseries(path, rows, columns=None):
    columns = columns or LOG_COLUMNS
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(",".join(columns) + "\n")
        for r in rows:
            vals = []
            for c in columns:
                v = r.get(c, None)
                if v is None:
                    vals.append("")          # not applicable (e.g. CMG in A group)
                elif isinstance(v, float):
                    vals.append("%.10g" % v)
                elif isinstance(v, (np.floating, np.integer)):
                    vals.append("%.10g" % float(v))
                else:
                    vals.append(str(v))
            fh.write(",".join(vals) + "\n")


def write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    def conv(o):
        if isinstance(o, dict):
            return {str(k): conv(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [conv(v) for v in o]
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return o

    with open(path, "w", encoding="utf-8") as fh:
        json.dump(conv(obj), fh, indent=2, ensure_ascii=False)


def write_run(out_dir, cl: ClosedLoop, metrics, cfg, extra_meta=None):
    from .config import dump
    os.makedirs(out_dir, exist_ok=True)
    write_timeseries(os.path.join(out_dir, "timeseries.csv"), cl.rows, cl.columns)
    write_json(os.path.join(out_dir, "metrics.json"), metrics)
    meta = dict(
        git_sha=_git("rev-parse HEAD"), git_branch=_git("branch --show-current"),
        python=sys.version, packages=_packages(),
        platform=platform.platform(),
        config=dump(cfg),
        experiment=dict(
            plant=cl.plant, case=getattr(cl.scn, "case", ""),
            modulation="removed (diagnostics only)",
            horizon_nodes=int(cl.ocp.N), horizon_s=float(cl.scn.rel_times[-1]),
            horizon_dts=list(map(float, cl.scn.dts)),
            grid_times=list(map(float, cl.scn.rel_times)),
            dt_ctrl=float(cl.dt), substeps=int(cl.substeps),
            nlp_vars=int(cl.ocp.n_vars), nlp_constraints=int(cl.ocp.n_cons),
            nx_model=int(cl.scn.nx), nu_model=int(cl.scn.nu),
            authority_scale=float(cl.scn.auth.scale),
            authority_mode=str(cl.scn.auth.mode),
            authority_calibration=cl.scn.authority_calibration,
            weights=cl.scn.w.as_dict(),
            gamma_limit_mode=str(cl.scn.gamma_limit_mode),
            gamma_limits_deg=(list(np.rad2deg(cl.scn.gamma_lim))
                              if cl.cmg_present and not cl.scn.gamma_unbounded
                              else "unbounded/none"),
            c_mg=(dict(h0=float(cl.scn.prm.cmg.h0),
                       gdot_max=float(cl.scn.prm.cmg.gdot_max),
                       gddot_max=float(cl.scn.prm.cmg.gddot_max),
                       tau_g=float(cl.scn.prm.cmg.tau_g))
                  if cl.cmg_present else "not applicable (no CMG)"),
            H_nom_mode=str(cl.scn.mod.h_nom), tau_d_mode=str(cl.scn.mod.tau_d),
            cmg_present=bool(cl.cmg_present),
            cmg_enabled=bool(cl.scn.cmg_enabled),
            seed=int(getattr(cl.scn, "seed", 0)),
            seed_randomize=bool(getattr(cl.scn, "seed_randomize", False)),
            dist_phase=[float(v) for v in np.asarray(
                getattr(cl.scn, "dist_phase", np.zeros(3)), float)],
            log_conventions=cl.log_conventions,
            columns=list(cl.columns),
            solver=dict(max_iter=cl.solver_cfg.max_iter, tol=cl.solver_cfg.tol,
                        linear_solver="mumps", engine="casadi-ipopt"),
        ),
    )
    if extra_meta:
        meta.update(extra_meta)
    write_json(os.path.join(out_dir, "meta.json"), meta)
    return meta
