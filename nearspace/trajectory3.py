"""Three-axis trajectory / attitude reference built on the skip profile.

The attitude reference is
    roll_ref  = man_roll(t)
    pitch_ref = gamma(t) + alpha_trim + man_pitch(t)
    yaw_ref   = heading(t)            + man_yaw(t)
where the commanded manoeuvre `man_*` is a quintic (zero rate and zero
acceleration at both ends, implementation plan §5.3)

    s = clip((t - t0)/Tm, 0, 1)
    man(t) = delta * (10 s^3 - 15 s^4 + 6 s^5)

and the disturbance is the constant/sinusoidal misalignment of the existing
model plus an optional smooth external torque pulse.
"""
from __future__ import annotations

import numpy as np

from .model6 import euler_to_quat, quat_conj, quat_mul, quat_normalize
from .trajectory import SkipTrajectory

QUINTIC_PEAK_RATE = 1.875      # max |ds/dt| of 10s^3-15s^4+6s^5  -> 1.875/Tm
QUINTIC_PEAK_ACC = 5.773503    # max |d2s/dt2|                  -> 5.7735/Tm^2
BUMP_PEAK_RATE = np.pi        # 0.5*(1-cos(2*pi*s))             -> pi/Tm
BUMP_PEAK_ACC = 2.0 * np.pi ** 2


class Maneuver:
    """Commanded attitude excursion on one body axis.

    shape='hold' : quintic step, the new attitude is held afterwards (§5.3)
    shape='bump' : raised cosine 0 -> delta -> 0 over Tm; this is the shape the
                   earlier C0-C4 / T1-T3 studies used, kept so those experiment
                   groups can be replayed with a different momentum device.
    """

    def __init__(self, axis="pitch", t0=1.0e9, duration=10.0, delta_deg=0.0,
                 shape="hold"):
        self.axis = str(axis).lower()
        self.shape = str(shape).lower()
        self.t0 = float(t0)
        self.Tm = max(float(duration), 1e-6)
        self.delta = np.deg2rad(float(delta_deg))

    @property
    def enabled(self) -> bool:
        return self.axis in ("roll", "pitch", "yaw") and abs(self.delta) > 0.0

    @property
    def t_end(self) -> float:
        return self.t0 + self.Tm

    @property
    def peak_rate(self) -> float:
        k = BUMP_PEAK_RATE if self.shape == "bump" else QUINTIC_PEAK_RATE
        return k * abs(self.delta) / self.Tm

    @property
    def peak_accel(self) -> float:
        k = BUMP_PEAK_ACC if self.shape == "bump" else QUINTIC_PEAK_ACC
        return k * abs(self.delta) / self.Tm ** 2

    def value(self, t: float) -> float:
        if not self.enabled:
            return 0.0
        s = (t - self.t0) / self.Tm
        if s <= 0.0 or s >= 1.0:
            return 0.0 if self.shape == "bump" else (0.0 if s <= 0.0 else self.delta)
        if self.shape == "bump":
            return self.delta * 0.5 * (1.0 - np.cos(2.0 * np.pi * s))
        return self.delta * (10.0 * s ** 3 - 15.0 * s ** 4 + 6.0 * s ** 5)

    def phase(self, t: float) -> int:
        """0 before, 1 during, 2 after."""
        if not self.enabled:
            return 0
        return 0 if t < self.t0 else (1 if t < self.t_end else 2)


class Trajectory3D:
    def __init__(self, cfg, atmosphere):
        self.cfg = cfg
        self.base = SkipTrajectory(cfg.trajectory, atmosphere)
        self.alpha_trim = np.deg2rad(float(cfg.trajectory.alpha_trim_deg))
        t3 = getattr(cfg, "trajectory3", None)
        self.traj = t3
        self.heading0 = np.deg2rad(float(getattr(t3, "heading_deg", 5.0)))
        self.heading_amp = np.deg2rad(float(getattr(t3, "heading_amp_deg", 3.0)))
        self.heading_period = float(getattr(t3, "heading_period", 200.0))
        self.dist_scale_qinf = bool(getattr(t3, "dist_scale_qinf", False))
        self.pulse_amp = np.array(getattr(t3, "dist_pulse_amp", [0.0, 0.0, 0.0]), dtype=float)
        self.pulse_start = float(getattr(t3, "dist_pulse_start", 1.0e9))
        self.pulse_dur = float(getattr(t3, "dist_pulse_duration", 1.0))
        m = getattr(t3, "maneuver", None)
        if m is None:
            specs = []
        elif isinstance(m, list):
            specs = m
        else:
            specs = [m]
        self.maneuvers = [
            Maneuver(axis=getattr(s, "axis", "pitch"),
                     t0=float(getattr(s, "start", 1.0e9)),
                     duration=float(getattr(s, "duration", 10.0)),
                     delta_deg=float(getattr(s, "delta_deg", 0.0)),
                     shape=str(getattr(s, "shape", "hold")))
            for s in specs]
        self.qinf_ref = 0.5 * self.rho(0.0) * self.V(0.0) ** 2
        # fixed high-altitude scoring window (threshold in km; 0 disables it)
        self.high_window_km = float(getattr(t3, "high_window_km", 90.0) or 0.0)
        self.dt_win = float(getattr(getattr(cfg, "simulation", None), "dt_ctrl", 0.2))

    @property
    def maneuver(self) -> Maneuver:
        """Primary (first) manoeuvre: kept for windows and single-axis reports."""
        if self.maneuvers:
            return self.maneuvers[0]
        return Maneuver(t0=1.0e9, delta_deg=0.0)

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
                + self.heading_amp * np.sin(2.0 * np.pi * t / self.heading_period))

    # ---- attitude reference ---------------------------------------------
    def _man(self, t):
        roll = pitch = yaw = 0.0
        for mv in self.maneuvers:
            v = mv.value(t)
            if mv.axis == "roll":
                roll += v
            elif mv.axis == "pitch":
                pitch += v
            elif mv.axis == "yaw":
                yaw += v
        return roll, pitch, yaw

    def roll_ref(self, t):
        return self._man(t)[0]

    def pitch_ref(self, t):
        return self.gamma(t) + self.alpha_trim + self._man(t)[1]

    def yaw_ref(self, t):
        return self.heading(t) + self._man(t)[2]

    def euler_ref(self, t):
        return np.array([self.roll_ref(t), self.pitch_ref(t), self.yaw_ref(t)])

    def q_ref(self, t):
        return quat_normalize(euler_to_quat(self.roll_ref(t), self.pitch_ref(t),
                                            self.yaw_ref(t)))

    def omega_ref(self, t, eps=2e-3):
        dq = quat_mul(quat_conj(self.q_ref(t - 0.5 * eps)), self.q_ref(t + 0.5 * eps))
        if dq[0] < 0.0:
            dq = -dq
        return 2.0 * dq[1:4] / eps

    # ---- environment ----------------------------------------------------
    def rho(self, t):
        return self.base.rho(t)

    def qinf(self, t):
        return 0.5 * self.rho(t) * self.V(t) ** 2

    def disturbance(self, t, cfg, prm):
        """Constant (gravity gradient / misalignment) plus aerodynamic part.

        With dist_scale_qinf the aerodynamic part scales with dynamic pressure,
        which is the physically consistent behaviour across a skip trajectory.
        """
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
        """Parameter vector p = [rho, V_n(3), M_d(3)]."""
        return np.concatenate([[self.rho(t)], self.V_ned(t),
                               self.disturbance(t, self.cfg, prm)])

    # ---- experiment windows ---------------------------------------------
    def windows(self, T):
        out = {}
        if self.pulse_start < T:
            out["dist"] = (self.pulse_start, min(self.pulse_start + self.pulse_dur, T))
        live = [mv for mv in self.maneuvers if mv.enabled and mv.t0 < T]
        if live:
            out["man"] = (min(mv.t0 for mv in live),
                          min(max(mv.t_end for mv in live), T))
        # the rarefied-arc window is defined by the trajectory alone (never by
        # the results), so A and B are scored over exactly the same interval
        if self.high_window_km > 0.0:
            tt = np.arange(0.0, T + 1e-9, self.dt_win)
            hh = np.array([self.h(x) for x in tt]) / 1000.0
            above = np.flatnonzero(hh >= self.high_window_km)
            if above.size:
                out["high"] = (float(tt[above[0]]), float(tt[above[-1]]))
        return out
