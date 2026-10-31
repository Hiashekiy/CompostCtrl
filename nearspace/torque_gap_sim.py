"""Closed-loop runner for the torque-gap modulated joint optimisation.

Two plants:

P0 (``plant: P0``)  the 11-state virtual-torque truth of nearspace.torque_gap_model.
                    The optimiser model and the truth model are the same 11-state
                    dynamics, so this plant tests the algorithm itself.
P1 (``plant: P1``)  the legacy 18-state truth of nearspace.model6c (real servo and
                    gimbal-rate loops).  The optimiser torque command tau_a is
                    converted to a bounded surface deflection by
                    nearspace.torque_gap_authority.allocate_deflection and the
                    achieved aerodynamic moment is logged separately.  P0 and P1
                    results are never mixed.

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
import time
from dataclasses import dataclass

import numpy as np

from .model6 import aero_coeffs, quat_to_R
from .model6c import (VehicleCMGParams, aero_moment6c, plant_step6c)
from .torque_gap_authority import allocate_deflection, natural_moment_v, surface_matrix
from .torque_gap_model import (attitude_error_vector, plant_step_tg, rk4_step_tg,
                               split_state)
from .torque_gap_modulation import capability_residual, nominal_demand

LOG_COLUMNS = (
    ["t", "h_km", "rho", "Q", "step"]
    + ["q%d" % i for i in range(4)] + ["qref%d" % i for i in range(4)]
    + ["wx", "wy", "wz", "wrefx", "wrefy", "wrefz"]
    + ["att_err_deg", "rate_err"]
    + ["g1", "g2", "g3", "g4", "nu1", "nu2", "nu3", "nu4",
       "nuc1", "nuc2", "nuc3", "nuc4"]
    + ["Hx", "Hy", "Hz", "H_norm", "sigma_min"]
    + ["tauc_cmd_x", "tauc_cmd_y", "tauc_cmd_z"]           # commanded tau_a
    + ["tau_act_x", "tau_act_y", "tau_act_z"]              # achieved tau_a
    + ["cmg_x", "cmg_y", "cmg_z"]                          # achieved -A nu
    + ["tau_nom_x", "tau_nom_y", "tau_nom_z"]
    + ["Rp_x", "Rp_y", "Rp_z", "Rm_x", "Rm_y", "Rm_z", "d_x", "d_y", "d_z"]
    + ["lo_x", "lo_y", "lo_z", "hi_x", "hi_y", "hi_z"]
    + ["E_gap", "gap_residual", "tau_bound_used", "nu_sat"]
    + ["d_pitch_plus_x", "d_pitch_minus_x"]                # deflection diagnostics
    + ["cpu_s", "iters", "viol", "fallback", "status"]
)


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


class ClosedLoop:
    def __init__(self, scn, ocp=None, plant="P0"):
        from .torque_gap_ocp import SolverConfig, TorqueGapOCP
        self.scn = scn
        self.ocp = ocp or TorqueGapOCP(scn)
        self.plant = str(plant).upper()
        self.cfg = scn.cfg
        self.solver_cfg = SolverConfig.from_config(scn.cfg)
        self.substeps = int(getattr(scn.cfg.torque_gap, "plant_substeps", 8))
        self.dt = float(scn.dt_ctrl)
        self.n_steps = int(round((scn.t_end - scn.t_start) / self.dt))
        self.vp = VehicleCMGParams.from_config(scn.cfg)
        from .cmg import CMGCluster
        self.cluster6c = CMGCluster(self.vp.cmg)      # P1 truth geometry
        self.delta = np.zeros(3)
        self.rows = []
        self.log = {}

    # ---- plant-specific execution ---------------------------------
    def _tau_d(self, t):
        return np.asarray(self.scn.traj.disturbance(t, self.cfg, self.scn.veh), float)

    def _step_P0(self, x, tau_a, nu, t):
        h = self.dt / self.substeps
        gamma_lim = self.scn.gamma_lim
        for i in range(self.substeps):
            td = self._tau_d(t + (i + 0.5) * h)
            u = np.concatenate([tau_a, nu])
            x = rk4_step_tg(x, u, td, h, self.scn.prm, self.scn.cluster)
            if not self.scn.gamma_unbounded:
                x[7:11] = np.clip(x[7:11], gamma_lim[0], gamma_lim[1])
        return x, tau_a.copy(), nu.copy()

    def _step_P1(self, x18, tau_a_des, nu_cmd, t):
        """18-state truth: torque -> bounded deflection, gimbal rate commanded."""
        q, w, gamma = split_state(x18)
        rho = float(self.scn.traj.rho(t))
        V_ned = np.asarray(self.scn.traj.V_ned(t), float)
        V = float(np.linalg.norm(V_ned))
        Q = 0.5 * rho * V ** 2
        p = np.concatenate([[rho], V_ned, self._tau_d(t)])
        xprobe = np.concatenate([q, w, np.zeros(3), gamma, np.zeros(4)])
        from .model6 import flow_angles
        alpha, beta, _ = flow_angles(xprobe, p)
        nat = natural_moment_v(Q, alpha, beta, w, V, self.scn.veh)
        M = surface_matrix(Q, self.scn.veh)
        span = np.asarray(self.scn.veh.delta_dot_max, float) * self.dt
        delta_cmd, res = allocate_deflection(np.asarray(tau_a_des) - nat, M,
                                             self.scn.veh, delta_prev=self.delta,
                                             ddelta_max=span)
        self.delta = delta_cmd
        u18 = np.concatenate([delta_cmd, nu_cmd])
        x18n, u_act = plant_step6c(x18, u18, p, self.dt, self.vp, self.cluster6c,
                                   substeps=self.substeps)
        # physical gimbal travel stops: the legacy 18-state truth has no travel
        # limit, so it is applied here exactly as in the P0 plant.  Without this
        # the truth gimbal angle runs away (|gamma| >> pi/2), the measured state
        # leaves the optimiser's gimbal box and every subsequent NLP is infeasible.
        if not self.scn.gamma_unbounded:
            lim = self.scn.gamma_lim
            x18n[10:14] = np.clip(x18n[10:14], lim[0], lim[1])
        tau_act = aero_moment6c(x18n, p, self.vp)
        return x18n, tau_act, u_act[3:7].copy(), np.asarray(delta_cmd, float), res

    def _init_plant(self):
        x = self.scn.initial_state()
        if self.plant == "P0":
            return x
        x18 = np.zeros(18)
        x18[0:4] = x[0:4]
        x18[4:7] = x[4:7]
        x18[10:14] = x[7:11]
        return x18

    def _state11(self, x):
        if self.plant == "P0":
            return x
        return np.concatenate([x[0:4], x[4:7], x[10:14]])

    # ---- fallback --------------------------------------------------
    def _fallback(self, plan, x, nu_prev, tau_prev):
        """Bounded stabilising action when the NLP result cannot be used.

        1. the first action of the previous feasible plan, re-checked against the
           current aerodynamic box, gimbal rate / acceleration and travel limits;
        2. if that is not admissible, a proportional attitude stabiliser with the
           gap filled by a bounded least-squares gimbal rate.
        """
        lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
        gdot = float(self.scn.prm.cmg.gdot_max)
        gddot = float(self.scn.prm.cmg.gddot_max)
        if self._last_action is not None:
            tau_a = np.clip(self._last_action[0:3], lo, hi)
            nu = np.clip(self._last_action[3:7], -gdot, gdot)
            nu = np.clip(nu, nu_prev - gddot * self.dt, nu_prev + gddot * self.dt)
            return np.concatenate([tau_a, nu]), "previous"
        q, w, _ = split_state(x)
        qref = plan["cond"]["q_ref"][0]
        wref = plan["cond"]["omega_ref"][0]
        eR = attitude_error_vector(qref, q)
        wdot = 2.0 / self.dt * (0.05 * eR - (w - wref))          # simple PD law
        tau_ff = nominal_demand(self.scn.traj, plan["cond"]["times"][0], self.scn.prm,
                                plan["mod"]["H_nom"], plan["cond"]["tau_d"][0])
        tau_a = np.clip(tau_ff + self.scn.prm.J @ wdot, lo, hi)
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
        x11 = self._state11(x)
        nu_prev = np.zeros(4)
        tau_prev = np.zeros(3)
        self._last_action = None
        warm = None
        gdot = float(scn.prm.cmg.gdot_max)
        gddot = float(scn.prm.cmg.gddot_max)
        gamma_lim = scn.gamma_lim
        self.rows = []
        for step in range(self.n_steps):
            t = scn.t_start + step * self.dt
            x11 = self._state11(x)
            # the optimiser's gimbal box must contain the measured configuration
            g_clip = (np.clip(x11[7:11], gamma_lim[0], gamma_lim[1])
                      if not scn.gamma_unbounded else x11[7:11])
            if np.any(g_clip != x11[7:11]):
                self.gamma_clipped_steps = getattr(self, "gamma_clipped_steps", 0) + 1
                x11 = x11.copy()
                x11[7:11] = g_clip
            plan = scn.plan(t, x11)
            res = ocp.solve(plan, x11, nu_prev, tau_prev, warm)
            fallback = 0
            ok = (res.ok and res.max_violation <= 1e-4
                  and res.cpu_s <= self.solver_cfg.max_cpu_s)
            if ok:
                u_cmd = res.u0.copy()
                status = res.status
                warm = (res.X, res.U, res.V)
            else:
                u_cmd, why = self._fallback(plan, x11, nu_prev, tau_prev)
                status = "%s|%s" % (res.status, why)
                fallback = 1
            # execution-side actuator limits (independent of the optimiser)
            lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
            tau_a_cmd = np.clip(u_cmd[0:3], lo, hi)
            nu_cmd = np.clip(u_cmd[3:7], -gdot, gdot)
            nu_cmd = np.clip(nu_cmd, nu_prev - gddot * self.dt, nu_prev + gddot * self.dt)
            if self.plant == "P0":
                x, tau_act, nu_act = self._step_P0(x, tau_a_cmd, nu_cmd, t)
                delta_cmd = np.zeros(3)
                alloc_res = np.zeros(3)
            else:
                x, tau_act, nu_act, delta_cmd, alloc_res = self._step_P1(
                    x, tau_a_cmd, nu_cmd, t)
            self._last_action = np.concatenate([tau_a_cmd, nu_cmd])
            nu_prev = np.asarray(nu_act, float)
            tau_prev = np.asarray(tau_act, float)
            self.rows.append(self._log_row(step, t, x, plan, res, tau_a_cmd, tau_act,
                                           nu_cmd, nu_act, delta_cmd, alloc_res,
                                           fallback, status))
        return self._summary()

    # ---- logging ---------------------------------------------------
    def _log_row(self, step, t, x, plan, res, tau_a_cmd, tau_act, nu_cmd, nu_act,
                 delta_cmd, alloc_res, fallback, status):
        x11 = self._state11(x)
        q, w, gamma = split_state(x11)
        t0 = plan["cond"]["times"][0]
        qref = plan["cond"]["q_ref"][0]
        wref = plan["cond"]["omega_ref"][0]
        eR = attitude_error_vector(qref, q)
        H = self.scn.cluster.momentum(gamma)
        A = self.scn.cluster.jacobian(gamma)
        cmg = -A @ nu_act
        d = plan["mod"]["d"][0]
        E = capability_residual(gamma, d, self.scn.cluster,
                               float(self.scn.prm.cmg.gdot_max))[0]
        gap_res = float(np.linalg.norm(cmg - d))
        lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
        tol = 1e-6 * np.maximum(1.0, np.abs(hi - lo))
        at_bound = np.mean((np.abs(tau_a_cmd - lo) <= tol) |
                           (np.abs(tau_a_cmd - hi) <= tol))
        row = dict(
            t=t, h_km=self.scn.traj.h(t) / 1000.0,
            rho=plan["cond"]["rho"][0], Q=plan["box"]["Q"][0], step=step,
            **{"q%d" % i: q[i] for i in range(4)},
            **{"qref%d" % i: qref[i] for i in range(4)},
            wx=w[0], wy=w[1], wz=w[2], wrefx=wref[0], wrefy=wref[1], wrefz=wref[2],
            att_err_deg=np.rad2deg(np.linalg.norm(eR)),
            rate_err=float(np.linalg.norm(w - wref)),
            **{"g%d" % (i + 1): gamma[i] for i in range(4)},
            **{"nu%d" % (i + 1): nu_act[i] for i in range(4)},
            **{"nuc%d" % (i + 1): nu_cmd[i] for i in range(4)},
            Hx=H[0], Hy=H[1], Hz=H[2], H_norm=float(np.linalg.norm(H)),
            sigma_min=self.scn.cluster.sigma_min(A),
            tauc_cmd_x=tau_a_cmd[0], tauc_cmd_y=tau_a_cmd[1], tauc_cmd_z=tau_a_cmd[2],
            tau_act_x=tau_act[0], tau_act_y=tau_act[1], tau_act_z=tau_act[2],
            cmg_x=cmg[0], cmg_y=cmg[1], cmg_z=cmg[2],
            **{("tau_nom_%s" % ax): plan["mod"]["tau_nom"][0][i]
               for i, ax in enumerate("xyz")},
            **{("Rp_%s" % ax): plan["mod"]["R_plus"][0][i] for i, ax in enumerate("xyz")},
            **{("Rm_%s" % ax): plan["mod"]["R_minus"][0][i] for i, ax in enumerate("xyz")},
            **{("d_%s" % ax): d[i] for i, ax in enumerate("xyz")},
            **{("lo_%s" % ax): lo[i] for i, ax in enumerate("xyz")},
            **{("hi_%s" % ax): hi[i] for i, ax in enumerate("xyz")},
            E_gap=E, gap_residual=gap_res, tau_bound_used=float(at_bound),
            nu_sat=float(np.mean(np.abs(nu_cmd) >= 0.999 * float(self.scn.prm.cmg.gdot_max))),
            d_pitch_plus_x=delta_cmd[0], d_pitch_minus_x=delta_cmd[1],
            cpu_s=res.cpu_s, iters=res.iter_count, viol=res.max_violation,
            fallback=fallback, status=status,
        )
        return row

    # ---- summary ---------------------------------------------------
    def _summary(self):
        rows = self.rows
        t = np.array([r["t"] for r in rows])
        err = np.array([r["att_err_deg"] for r in rows])
        cpu = np.array([r["cpu_s"] for r in rows])
        fb = np.array([r["fallback"] for r in rows])
        scn = self.scn
        man = scn.traj.maneuvers
        win = (min(m.t0 for m in man if m.enabled),
               max(m.t_end for m in man if m.enabled)) if any(m.enabled for m in man) else None
        pre = (win[0] - 12.0, win[0]) if win else None
        masks = dict(
            all=np.ones_like(t, bool),
            pre=((t >= pre[0]) & (t < pre[1])) if pre else np.zeros_like(t, bool),
            maneuver=((t >= win[0]) & (t <= win[1])) if win else np.zeros_like(t, bool),
            post=((t > win[1])) if win else np.zeros_like(t, bool),
        )
        out = dict(
            plant=self.plant, mode=scn.mode,
            t_start=scn.t_start, t_end=scn.t_end, dt_ctrl=self.dt,
            n_steps=len(rows),
            window=list(win) if win else None,
            rmse_deg={k: float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else None
                      for k, m in masks.items()},
            max_err_deg={k: float(np.max(err[m])) if m.any() else None
                         for k, m in masks.items()},
            cpu_s=dict(mean=float(np.mean(cpu)), p95=float(np.percentile(cpu, 95)),
                       max=float(np.max(cpu)), total=float(np.sum(cpu))),
            fallback_steps=int(fb.sum()),
            fallback_ratio=float(fb.mean()),
            iterations=dict(mean=float(np.mean([r["iters"] for r in rows])),
                            max=int(np.max([r["iters"] for r in rows]))),
            max_violation=float(np.max([r["viol"] for r in rows])),
            tau_bound_used_mean=float(np.mean([r["tau_bound_used"] for r in rows])),
            H_norm_max=float(max(r["H_norm"] for r in rows)),
            sigma_min_min=float(min(r["sigma_min"] for r in rows)),
            status_counts={s: int(sum(1 for r in rows if r["status"] == s))
                           for s in sorted({r["status"] for r in rows})},
        )
        if win:
            mw = masks["maneuver"]
            i0 = int(np.argmax(t >= win[0]))
            out["gamma_at_maneuver_start"] = [float(rows[i0]["g%d" % (i + 1)])
                                              for i in range(4)]
            if mw.any():
                out["nu_saturation_ratio_maneuver"] = float(
                    np.mean([rows[i]["nu_sat"] for i in range(len(rows)) if mw[i]]))
                out["E_gap_maneuver_mean"] = float(np.mean(
                    [rows[i]["E_gap"] for i in range(len(rows)) if mw[i]]))
                out["gap_residual_maneuver_mean"] = float(np.mean(
                    [rows[i]["gap_residual"] for i in range(len(rows)) if mw[i]]))
                out["sigma_min_maneuver_min"] = float(min(
                    rows[i]["sigma_min"] for i in range(len(rows)) if mw[i]))
            # configuration migration: gimbal deviation from the initial
            # configuration and the gimbal work spent *before* the manoeuvre
            g0 = np.array([rows[0]["g%d" % (i + 1)] for i in range(4)])
            dev = np.array([max(abs(rows[i]["g%d" % (k + 1)] - g0[k])
                                for k in range(4)) for i in range(len(rows))])
            prem = t < win[0]
            nus = np.array([max(abs(r["nu%d" % (i + 1)]) for i in range(4)) for r in rows])
            out["pre_gimbal_travel_rad"] = float(np.sum(nus[prem]) * self.dt)
            out["pre_H_norm_at_start"] = float(rows[i0]["H_norm"])
            out["gamma_dev_max_deg"] = float(np.rad2deg(np.max(dev)))
            lead = np.flatnonzero((dev > 0.05) & prem)             # > ~3 deg
            out["migration_onset_t"] = float(t[lead[0]]) if lead.size else None
            out["migration_lead_s"] = float(win[0] - t[lead[0]]) if lead.size else 0.0
            # manoeuvre completion: last sample above 1 deg, relative to t0
            bad = np.flatnonzero((err > 1.0) & (t >= win[0]))
            out["settle_time_s"] = (float(t[bad[-1]] - win[0] + self.dt)
                                    if bad.size else 0.0)
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
                v = r.get(c, "")
                if isinstance(v, float):
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
    write_timeseries(os.path.join(out_dir, "timeseries.csv"), cl.rows)
    write_json(os.path.join(out_dir, "metrics.json"), metrics)
    meta = dict(
        git_sha=_git("rev-parse HEAD"), git_branch=_git("branch --show-current"),
        python=sys.version, packages=_packages(),
        platform=platform.platform(),
        config=dump(cfg),
        experiment=dict(
            mode=cl.scn.mode, plant=cl.plant, case=getattr(cl.scn, "case", ""),
            lambda_cfg=(0.0 if cl.scn.mode == "B0" else cl.scn.lambda_cfg),
            horizon_nodes=int(cl.ocp.N), horizon_s=float(cl.scn.rel_times[-1]),
            horizon_dts=list(map(float, cl.scn.dts)),
            grid_times=list(map(float, cl.scn.rel_times)),
            dt_ctrl=float(cl.dt), substeps=int(cl.substeps),
            nlp_vars=int(cl.ocp.n_vars), nlp_constraints=int(cl.ocp.n_cons),
            authority_scale=float(cl.scn.auth.scale),
            authority_mode=str(cl.scn.auth.mode),
            authority_calibration=cl.scn.authority_calibration,
            weights=cl.scn.w.as_dict(),
            gamma_limit_mode=str(cl.scn.gamma_limit_mode),
            gamma_limits_deg=(list(np.rad2deg(cl.scn.gamma_lim))
                              if not cl.scn.gamma_unbounded else "unbounded"),
            c_mg=dict(h0=float(cl.scn.prm.cmg.h0),
                      gdot_max=float(cl.scn.prm.cmg.gdot_max),
                      gddot_max=float(cl.scn.prm.cmg.gddot_max),
                      tau_g=float(cl.scn.prm.cmg.tau_g)),
            H_nom_mode=str(cl.scn.mod.h_nom), tau_d_mode=str(cl.scn.mod.tau_d),
            solver=dict(max_iter=cl.solver_cfg.max_iter, tol=cl.solver_cfg.tol,
                        linear_solver="mumps", engine="casadi-ipopt"),
            seed=int(getattr(cfg.torque_gap, "seed", 0)),
        ),
    )
    if extra_meta:
        meta.update(extra_meta)
    write_json(os.path.join(out_dir, "meta.json"), meta)
    return meta
