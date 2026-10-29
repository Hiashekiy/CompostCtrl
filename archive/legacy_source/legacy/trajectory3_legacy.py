"""Frozen copy of the ORIGINAL three-axis trajectory used by C0-C4 / T1-T3.

The current `nearspace/trajectory3.py` replaces the raised-cosine
`pitch_amp_deg / roll_amp_deg / yaw_amp_deg` pulses by an explicit `maneuver`
list.  The archived C/T scripts must keep the old semantics to reproduce the
numbers in `results/legacy_rw/`, so they import this file instead.
"""
from __future__ import annotations

import numpy as np

from nearspace.model6 import euler_to_quat, quat_conj, quat_mul, quat_normalize
from nearspace.trajectory import SkipTrajectory


class Trajectory3D:
    def __init__(self, cfg, atmosphere):
        self.cfg = cfg
        self.base = SkipTrajectory(cfg.trajectory, atmosphere)
        self.alpha_trim = np.deg2rad(float(cfg.trajectory.alpha_trim_deg))
        t3 = getattr(cfg, "trajectory3", None)
        self.heading0 = np.deg2rad(float(getattr(t3, "heading_deg", 5.0)))
        self.heading_amp = np.deg2rad(float(getattr(t3, "heading_amp_deg", 3.0)))
        self.heading_period = float(getattr(t3, "heading_period", 200.0))
        self.pitch_amp = np.deg2rad(float(getattr(t3, "pitch_amp_deg", 5.0)))
        self.roll_amp = np.deg2rad(float(getattr(t3, "roll_amp_deg", 10.0)))
        self.yaw_amp = np.deg2rad(float(getattr(t3, "yaw_amp_deg", 4.0)))
        self.man_start = float(getattr(t3, "man_start", 105.0))
        self.man_dur = float(getattr(t3, "man_duration", 20.0))
        self.dist_scale_qinf = bool(getattr(t3, "dist_scale_qinf", False))
        self.pulse_amp = np.array(getattr(t3, "dist_pulse_amp", [0.0, 0.0, 0.0]), dtype=float)
        self.pulse_start = float(getattr(t3, "dist_pulse_start", 1.0e9))
        self.pulse_dur = float(getattr(t3, "dist_pulse_duration", 1.0))
        self.qinf_ref = 0.5 * self.rho(0.0) * self.V(0.0) ** 2
        self.traj = t3

    # ---- geometry -------------------------------------------------------
    def h(self, t):
        return self.base.h(t)

    def hdot(self, t):
        return self.base.hdot(t)

    def V(self, t):
        return self.base.V(t)

    def gamma(self, t):
        return self.base.gamma(t)

    def heading(self, t):
        return (self.heading0
                + self.heading_amp * np.sin(2.0 * np.pi * t / self.heading_period)
                + self.yaw_amp * self._pulse(t))

    def _pulse(self, t, delay=0.0):
        t0, dur = self.man_start + delay, self.man_dur
        if t <= t0 or t >= t0 + dur:
            return 0.0
        return 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - t0) / dur))

    def roll_ref(self, t):
        return self.roll_amp * self._pulse(t)

    def pitch_ref(self, t):
        return self.gamma(t) + self.alpha_trim + self.pitch_amp * self._pulse(t)

    def yaw_ref(self, t):
        return self.heading(t)

    # ---- attitude reference --------------------------------------------
    def q_ref(self, t):
        return quat_normalize(euler_to_quat(self.roll_ref(t), self.pitch_ref(t),
                                            self.yaw_ref(t)))

    def omega_ref(self, t, eps=1e-3):
        dq = quat_mul(quat_conj(self.q_ref(t)), self.q_ref(t + eps))
        if dq[0] < 0.0:
            dq = -dq
        return 2.0 * dq[1:4] / eps

    # ---- environment ----------------------------------------------------
    def rho(self, t):
        return self.base.rho(t)

    def qinf(self, t):
        return 0.5 * self.rho(t) * self.V(t) ** 2

    def disturbance(self, t, cfg, prm):
        base = prm.Md0 + prm.Md_amp * np.sin(2.0 * np.pi * prm.Md_freq * t)
        if self.dist_scale_qinf:
            s = self.qinf(t) / max(self.qinf_ref, 1e-9)
            base = prm.Md0 + (base - prm.Md0) * s
        if self.pulse_start <= t <= self.pulse_start + self.pulse_dur:
            s = 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - self.pulse_start) / self.pulse_dur))
            base = base + self.pulse_amp * s
        return base

    def V_ned(self, t):
        V, g, chi = self.V(t), self.gamma(t), self.heading(t)
        return V * np.array([np.cos(g) * np.cos(chi), np.cos(g) * np.sin(chi), -np.sin(g)])

    def p_array(self, t, prm):
        return np.concatenate([[self.rho(t)], self.V_ned(t),
                               self.disturbance(t, self.cfg, prm)])
