"""Scenario assembly for the torque-gap experiments (shared by all entry points).

Owns the arithmetic that every entry point needs and must not duplicate:
prediction grid, reference flight conditions, aerodynamic torque boxes,
modulation vectors and the initial state.  Everything physical (vehicle inertia,
aerodynamic coefficients, CMG geometry, atmosphere, skip trajectory) is taken
from the frozen repository modules unchanged.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .atmosphere_factory import make_atmosphere
from .config import load_config
from .model6 import euler_to_quat, quat_normalize
from .torque_gap_authority import AuthorityConfig, build_boxes
from .torque_gap_model import TorquePlantParams, pack_state
from .torque_gap_modulation import (ModulationConfig, gap_by_axis_onset,
                                    gap_onset_time, modulation, nominal_demand)
from .trajectory3 import Trajectory3D


# --------------------------------------------------------------------------
# configuration helpers
# --------------------------------------------------------------------------
def tg_cfg(cfg, *path, default=None):
    """Nested tolerant getattr for the `torque_gap:` block."""
    node = getattr(cfg, "torque_gap", None)
    for key in path:
        if node is None:
            return default
        node = getattr(node, key, None)
    return default if node is None else node


@dataclass
class Weights:
    e_scale: float = np.deg2rad(1.0)
    omega_scale: float = 0.02
    d_scale: float = 1.0
    nu_scale: float = 1.5
    w_att: float = 1.0
    w_rate: float = 1.0
    q_terminal: float = 10.0
    r_nu: float = 1e-3
    r_dnu: float = 1e-2
    r_dtau: float = 1e-4

    @classmethod
    def from_config(cls, cfg):
        w = getattr(cfg.torque_gap, "weights", None) if hasattr(cfg, "torque_gap") else None
        if w is None:
            return cls()

        def g(name, default):
            return float(getattr(w, name, default))

        return cls(e_scale=np.deg2rad(g("e_scale_deg", 1.0)),
                   omega_scale=g("omega_scale", 0.02),
                   d_scale=g("d_scale", 1.0),
                   nu_scale=g("nu_scale", 1.5),
                   w_att=g("w_att", 1.0),
                   w_rate=g("w_rate", 1.0),
                   q_terminal=g("q_terminal", 10.0),
                   r_nu=g("r_nu", 1e-3),
                   r_dnu=g("r_dnu", 1e-2),
                   r_dtau=g("r_dtau", 1e-4))

    def as_dict(self):
        return dict(e_scale_deg=float(np.rad2deg(self.e_scale)),
                    omega_scale=self.omega_scale, d_scale=self.d_scale,
                    nu_scale=self.nu_scale, w_att=self.w_att, w_rate=self.w_rate,
                    q_terminal=self.q_terminal, r_nu=self.r_nu,
                    r_dnu=self.r_dnu, r_dtau=self.r_dtau)


def prediction_grid(cfg):
    """(times relative to the current step, dt array) of the shooting grid."""
    hz = tg_cfg(cfg, "horizon", default=None)
    if hz is None:
        hz = [dict(n=6, dt=0.2), dict(n=4, dt=0.5), dict(n=4, dt=1.0),
              dict(n=8, dt=2.0)]
    dts = []
    for blk in hz:
        n = int(getattr(blk, "n", blk["n"] if isinstance(blk, dict) else 1))
        dt = float(getattr(blk, "dt", blk["dt"] if isinstance(blk, dict) else 0.2))
        dts.extend([dt] * n)
    dts = np.asarray(dts, float)
    times = np.concatenate([[0.0], np.cumsum(dts)])
    return times, dts


# --------------------------------------------------------------------------
# scenario
# --------------------------------------------------------------------------
class Scenario:
    def __init__(self, cfg):
        self.cfg = cfg
        self.atm = make_atmosphere(cfg)
        self.traj = Trajectory3D(cfg, self.atm)
        self.prm = TorquePlantParams.from_config(cfg)
        self.cluster = self.prm.cluster()
        self.veh = self.prm.veh
        self.w = Weights.from_config(cfg)
        self.auth = AuthorityConfig.from_config(cfg)
        self.mod = ModulationConfig.from_config(cfg)
        self.authority_calibration = None
        self.rel_times, self.dts = prediction_grid(cfg)
        self.t_start = float(tg_cfg(cfg, "start", default=0.0))
        self.t_end = float(tg_cfg(cfg, "end", default=100.0))
        self.dt_ctrl = float(tg_cfg(cfg, "dt_ctrl", default=0.2))
        self.mode = str(tg_cfg(cfg, "mode", default="B0")).upper()
        self.lambda_cfg = float(tg_cfg(cfg, "lambda_cfg", default=1.0))
        self.gamma_lim = (np.deg2rad(float(tg_cfg(cfg, "gamma_min_deg", default=-90.0))),
                          np.deg2rad(float(tg_cfg(cfg, "gamma_max_deg", default=90.0))))
        self.tau_rate_limit = tg_cfg(cfg, "tau_rate_limit", default=None)
        self.rho_fixed = None
        if str(self.auth.mode).lower() == "fixed":
            self.rho_fixed = float(self.atm.rho(self.auth.fixed_h_km * 1000.0))
        if self.auth.calibrate:
            self._calibrate_authority()

    # ---- authority calibration -----------------------------------------
    def _calibrate_authority(self):
        """Offline check that the box is realisable by the real surfaces.

        The box is the per-axis hull of the reachable set; random interior targets
        are only *simultaneously* realisable if the hull is shrunk.  The largest
        uniform scale that keeps at least `require_frac` of the sampled targets
        realisable is used, and the whole calibration is written to the run meta.
        """
        from .torque_gap_authority import calibrate_scale

        Qs = [float(0.5 * self.traj.rho(t) * self.traj.V(t) ** 2)
              for t in np.linspace(self.t_start, self.t_end, 9)]
        Qs = sorted(set(round(q, 6) for q in Qs))
        scale, diag = calibrate_scale(self.veh, Qs,
                                      require_frac=self.auth.require_frac,
                                      n_samples=self.auth.n_samples,
                                      tol_frac=self.auth.tol_frac)
        self.auth.scale = float(scale)
        self.authority_calibration = dict(scale=float(scale), Q_samples=Qs,
                                          steps=diag,
                                          require_frac=self.auth.require_frac,
                                          n_samples=self.auth.n_samples)

    # ---- reference / environment ---------------------------------------
    def abs_times(self, t0):
        return t0 + self.rel_times

    def cond(self, times):
        """Reference flight condition at the given absolute times."""
        times = np.atleast_1d(np.asarray(times, float))
        rho = np.array([self.traj.rho(t) for t in times])
        V = np.array([self.traj.V(t) for t in times])
        V_ned = np.array([self.traj.V_ned(t) for t in times])
        q_ref = np.array([self.traj.q_ref(t) for t in times])
        omega_ref = np.array([self.traj.omega_ref(t) for t in times])
        h_m = np.array([self.traj.h(t) for t in times])
        tau_d = np.array([self.traj.disturbance(t, self.cfg, self.veh) for t in times])
        return dict(times=times, rho=rho, V=V, V_ned=V_ned, q_ref=q_ref,
                    omega_ref=omega_ref, h_m=h_m, tau_d=tau_d)

    def boxes(self, cond):
        return build_boxes(cond["times"], cond, self.veh, self.auth,
                           rho_override=self.rho_fixed)

    def demand(self, cond, H_nom):
        K = cond["times"].size
        out = np.zeros((K, 3))
        for k in range(K):
            tau_d = cond["tau_d"][k] if self.mod.tau_d == "nominal" else np.zeros(3)
            out[k] = nominal_demand(self.traj, cond["times"][k], self.prm,
                                    H_nom, tau_d)
        return out

    def modulation_sequence(self, tau_nom, box):
        K = tau_nom.shape[0]
        R_plus = np.zeros((K, 3))
        R_minus = np.zeros((K, 3))
        d = np.zeros((K, 3))
        for k in range(K):
            m = modulation(tau_nom[k], box["tau_min"][k], box["tau_max"][k])
            R_plus[k], R_minus[k], d[k] = m["R_plus"], m["R_minus"], m["d"]
        return dict(tau_nom=tau_nom, R_plus=R_plus, R_minus=R_minus, d=d)

    def plan(self, t0, x=None):
        """Everything the optimiser needs at one control step."""
        cond = self.cond(self.abs_times(t0))
        box = self.boxes(cond)
        gamma_now = x[7:11] if x is not None else self.initial_state()[2]
        H_nom = self.cluster.momentum(gamma_now) if self.mod.h_nom == "freeze" \
            else np.zeros(3)
        tau_nom = self.demand(cond, H_nom)
        modul = self.modulation_sequence(tau_nom, box)
        modul["H_nom"] = H_nom
        return dict(cond=cond, box=box, mod=modul)

    # ---- initial state --------------------------------------------------
    def initial_state(self):
        t = self.t_start
        q = self.traj.q_ref(t)
        off = np.deg2rad(np.asarray(tg_cfg(self.cfg, "init", "attitude_offset_deg",
                                           default=[0.0, 0.0, 0.0]), float))
        if np.any(off != 0.0):
            from .model6 import quat_mul
            dq = euler_to_quat(off[0], off[1], off[2])
            q = quat_normalize(quat_mul(dq, q))
        w = tg_cfg(self.cfg, "init", "omega0", default="trim")
        if isinstance(w, str) and w.lower() == "trim":
            w = np.asarray(self.traj.omega_ref(t), float)
        else:
            w = np.asarray(w, float)
        g = np.deg2rad(np.asarray(tg_cfg(self.cfg, "init", "gamma0_deg",
                                         default=[0.0, 0.0, 0.0, 0.0]), float))
        return pack_state(q, w, g)

    # ---- diagnostics ----------------------------------------------------
    def describe(self, t0=0.0):
        x = self.initial_state()
        p = self.plan(self.t_start, x)
        times = p["cond"]["times"] - self.t_start
        info = dict(
            grid_nodes=int(self.rel_times.size),
            grid_steps=int(self.dts.size),
            horizon_s=float(self.rel_times[-1]),
            tau_min=p["box"]["tau_min"], tau_max=p["box"]["tau_max"],
            tau_nom=p["mod"]["tau_nom"], R_plus=p["mod"]["R_plus"],
            R_minus=p["mod"]["R_minus"], d=p["mod"]["d"],
            Q=p["box"]["Q"], times=times,
            gap_onset=gap_onset_time(times, p["mod"]["d"], self.mod.gap_tol),
            gap_onset_axis=gap_by_axis_onset(times, p["mod"]["d"], self.mod.gap_tol),
        )
        return info


def load_scenario(path):
    cfg = load_config(path)
    return Scenario(cfg), cfg
