"""Analytic near-space skip trajectory and attitude reference.

Replace the analytic profile by the real skip trajectory table if available;
everything downstream only uses h(t), V(t), gamma(t).
"""
from __future__ import annotations

import numpy as np


class SkipTrajectory:
    def __init__(self, cfg, atmosphere):
        self.cfg = cfg
        self.atm = atmosphere
        self.h0 = float(cfg.h0)
        self.amp = float(cfg.amp)
        self.period = float(cfg.period)
        self.V0 = float(cfg.V0)
        self.dVdt = float(cfg.dVdt)
        self.alpha_trim = np.deg2rad(float(cfg.alpha_trim_deg))
        self.man_amp = np.deg2rad(float(cfg.maneuver_amp_deg))
        self.man_start = float(cfg.maneuver_start)
        self.man_dur = float(cfg.maneuver_duration)

    # ---- geometry -------------------------------------------------------
    def h(self, t: float) -> float:
        return self.h0 + 0.5 * self.amp * (1.0 - np.cos(2.0 * np.pi * t / self.period))

    def hdot(self, t: float) -> float:
        return 0.5 * self.amp * (2.0 * np.pi / self.period) * np.sin(2.0 * np.pi * t / self.period)

    def V(self, t: float) -> float:
        return max(self.V0 - self.dVdt * t, 500.0)

    def gamma(self, t: float) -> float:
        return float(np.arctan2(self.hdot(t), self.V(t)))

    # ---- environment / reference ---------------------------------------
    def rho(self, t: float) -> float:
        return self.atm.rho(self.h(t))

    def maneuver(self, t: float) -> float:
        """Smooth C1 pitch pulse used to load the actuators in thin air."""
        t0, dur = self.man_start, self.man_dur
        if t <= t0 or t >= t0 + dur:
            return 0.0
        return 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - t0) / dur))

    def theta_ref(self, t: float) -> float:
        return self.gamma(t) + self.alpha_trim + self.man_amp * self.maneuver(t)

    def q_ref(self, t: float, eps: float = 1e-4) -> float:
        return (self.theta_ref(t + eps) - self.theta_ref(t - eps)) / (2.0 * eps)

    def qdot_ref(self, t: float, eps: float = 1e-3) -> float:
        return (self.q_ref(t + eps) - self.q_ref(t - eps)) / (2.0 * eps)

    def disturbance(self, t: float, cfg) -> float:
        return float(cfg.Md0 + cfg.Md_amp * np.sin(2.0 * np.pi * cfg.Md_freq * t))

    def p_array(self, t: float, dist_cfg) -> np.ndarray:
        return np.array([self.rho(t), self.V(t), self.gamma(t),
                         self.disturbance(t, dist_cfg)], dtype=float)
