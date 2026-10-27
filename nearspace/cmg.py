"""Single-gimbal CMG cluster (4-unit pyramid) with realistic limits.

Geometry (body axes, x forward / y right / z down)
    gimbal axes   g_i = [sin(b) cos(t_i), sin(b) sin(t_i), cos(b)]
    spin axes     s_i(0) = [-sin(t_i), cos(t_i), 0]          (perpendicular to g_i)
    s_i(g) = cos(g) s_i(0) + sin(g) (g_i x s_i(0))
    h_i(g) = h0 * s_i(g)                                     (rotor speed regulated)
    cluster momentum  H(g) = sum_i h_i(g)
    Jacobian          A(g) = dh/dg  (3x4),  A[:,i] = h0 (g_i x s_i(g))
    body torque       M = -A(g) gdot

With b = 54.7356 deg (cos^2 b = 1/3) every pair of gimbal axes satisfies
|g_i . g_j| = 1/3 (+1/3 for units 90 deg apart in azimuth, -1/3 for opposite
ones), i.e. they are orthogonal-complementary and span the three body axes.

Realistic limit set (see config.yaml 'cmg:' section)
    |gdot_i| <= gdot_max        gimbal rate limit
    |gddot_i| <= gddot_max      gimbal acceleration limit
    gdot_c reachability         |gdot_c,i - gdot_i| <= gddot_max * tau_g
    rotor speed regulated        h0 constant (approx; ~1% speed loop)
Singularity is measured by sigma_min(A) (equivalently det(A A^T)); the steering
must keep sigma_min above a floor.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

BETA_PYRAMID = np.arcsin(np.sqrt(2.0 / 3.0))   # 54.7356 deg


def _cross3(a, b):
    """Cross product of two (3,n) stacks -> (3,n)."""
    return np.vstack([a[1] * b[2] - a[2] * b[1],
                      a[2] * b[0] - a[0] * b[2],
                      a[0] * b[1] - a[1] * b[0]])


@dataclass
class CMGParams:
    n: int = 4
    beta: float = BETA_PYRAMID
    h0: float = 25.0            # N m s per unit rotor momentum
    gdot_max: float = 1.5       # rad/s   (~86 deg/s)
    gddot_max: float = 10.0     # rad/s^2
    tau_g: float = 0.03         # s  gimbal rate loop time constant
    rotor_rpm: float = 6000.0
    mass_per_unit: float = 20.0  # kg
    power_per_unit: float = 150.0  # W (windage + gimbal + electronics)
    sigma_floor_frac: float = 0.30     # floor as a fraction of the home sigma_min

    @classmethod
    def from_config(cls, cfg, **over):
        c = getattr(cfg, "cmg", None)
        kw = {}
        if c is not None:
            for k in ("n", "beta_deg", "h0", "gdot_max", "gddot_max", "tau_g",
                      "rotor_rpm", "mass_per_unit", "power_per_unit", "sigma_floor_frac"):
                if hasattr(c, k):
                    kw[k] = float(getattr(c, k))
            if "beta_deg" in kw:
                kw["beta"] = np.deg2rad(kw.pop("beta_deg"))
        kw.update(over)
        return cls(**{k: (int(v) if k == "n" else v) for k, v in kw.items()})

    @property
    def h_total(self) -> float:
        return self.n * self.h0

    @property
    def home_sigma_min(self) -> float:
        """sigma_min of the Jacobian at the home configuration (gamma = 0)."""
        return 0.816496580927726 * self.h0

    @property
    def sigma_min_home(self) -> float:
        """sigma_min of the unit Jacobian at home (A = h0 * unit jacobian)."""
        return self.home_sigma_min

    @property
    def sigma_floor(self) -> float:
        return self.sigma_floor_frac * self.home_sigma_min

    @property
    def torque_per_unit(self) -> float:
        return self.h0 * self.gdot_max


class CMGCluster:
    def __init__(self, prm: CMGParams):
        self.prm = prm
        n = prm.n
        th = 2.0 * np.pi * np.arange(n) / n
        sb, cb = np.sin(prm.beta), np.cos(prm.beta)
        self.g_axis = np.vstack([sb * np.cos(th), sb * np.sin(th), cb * np.ones(n)])  # (3,n)
        s0 = np.vstack([-np.sin(th), np.cos(th), np.zeros(n)])                        # (3,n)
        self.s0 = s0
        self.gxs0 = _cross3(self.g_axis, s0)              # 3xn = g x s0

    # ---- kinematics -----------------------------------------------------
    def spin_axes(self, gamma):
        g = np.asarray(gamma, dtype=float)
        return self.s0 * np.cos(g) + self.gxs0 * np.sin(g)      # 3xn

    def momentum(self, gamma):
        return self.prm.h0 * self.spin_axes(gamma).sum(axis=1)  # 3

    def jacobian(self, gamma):
        """A = dh/dgamma (3xn), A[:,i] = h0 (g_i x s_i)."""
        s = self.spin_axes(gamma)
        return self.prm.h0 * _cross3(self.g_axis, s)

    def torque(self, gamma, gdot):
        """Body torque produced by the CMGs (M = -A gdot)."""
        return -self.jacobian(gamma) @ np.asarray(gdot, dtype=float)

    @staticmethod
    def torque_capability(A):
        """Peak |M| for |gdot_i| <= 1 (scale by gdot_max) along the best direction."""
        # max over directions of ||A u|| with |u_i|<=1 -> sample the 2^4 vertices
        n = A.shape[1]
        verts = np.array([[(-1) ** ((k >> i) & 1) for i in range(n)]
                          for k in range(2 ** n)], dtype=float)
        return float(np.max(np.linalg.norm(A @ verts.T, axis=0)))

    # ---- singularity ----------------------------------------------------
    @staticmethod
    def sigma_min(A):
        return float(np.linalg.svd(A, compute_uv=False)[-1])

    @staticmethod
    def det_AAt(A):
        return float(np.linalg.det(A @ A.T))

    def sigma_min_grad(self, gamma):
        """d sigma_min / d gamma (finite difference; 4 components)."""
        g0 = np.asarray(gamma, dtype=float)
        out = np.zeros(self.prm.n)
        for i in range(self.prm.n):
            h = 1e-6
            gp = g0.copy(); gp[i] += h
            gm = g0.copy(); gm[i] -= h
            out[i] = (self.sigma_min(self.jacobian(gp)) - self.sigma_min(self.jacobian(gm))) / (2 * h)
        return out

    def singularity_grad(self, gamma):
        """d det(A A^T) / d gamma  (uphill = away from singularity)."""
        g0 = np.asarray(gamma, dtype=float)
        out = np.zeros(self.prm.n)
        for i in range(self.prm.n):
            h = 1e-6
            gp = g0.copy(); gp[i] += h
            gm = g0.copy(); gm[i] -= h
            out[i] = (self.det_AAt(self.jacobian(gp)) - self.det_AAt(self.jacobian(gm))) / (2 * h)
        return out

    # ---- realistic spec sheet ------------------------------------------
    def spec_sheet(self) -> dict:
        p = self.prm
        omega = p.rotor_rpm * 2 * np.pi / 60.0
        Jr = p.h0 / omega
        return dict(
            units=p.n,
            momentum_per_unit_Nms=p.h0,
            cluster_momentum_Nms=p.h_total,
            gimbal_rate_max_rad_s=p.gdot_max,
            gimbal_accel_max_rad_s2=p.gddot_max,
            torque_per_unit_Nm=p.torque_per_unit,
            rotor_rpm=p.rotor_rpm,
            rotor_inertia_kgm2=Jr,
            cluster_mass_kg=p.n * p.mass_per_unit,
            cluster_power_W=p.n * p.power_per_unit,
            home_sigma_min_Nms=p.home_sigma_min,
            sigma_floor_Nms=p.sigma_floor,
        )
