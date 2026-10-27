"""Real-atmosphere backends for the near-space attitude-control study.

Two interchangeable backends are exposed through a single interface that is
identical to :class:`nearspace.atmosphere.Atmosphere` (scalar queries returning
``float``), so :meth:`nearspace.trajectory.SkipTrajectory.p_array` can switch
between them without touching any other module:

``backend="msis2"`` (default)
    NRLMSIS 2.0 evaluated through :mod:`pymsis`.  The model is sampled once onto
    a dense altitude grid and cached as a CSV table in ``results/``; queries are
    log-linear (density) / linear (temperature) interpolations of that table.

``backend="us76"``
    The analytic U.S. Standard Atmosphere 1976: the seven temperature-gradient
    layers of the lower atmosphere (geopotential formulation, 0-86 km) plus the
    standard's analytic thermosphere approximation (86-1000 km).

Both backends provide ``rho``, ``T``, ``a``, ``p``, ``mean_free_path`` and
``knudsen`` (scalars) as well as ``*_array`` vectorised variants.

Conventions
-----------
* SI units everywhere; altitudes are **geometric** metres unless a name says
  ``_km``.
* ``p`` is the true static pressure: for MSIS2 it is rebuilt from the NRLMSIS
  composition (``p = n k_B T``), for US76 from the hydrostatic integration.
* ``a = sqrt(gamma * p / rho)`` with ``gamma = 1.4`` (frozen diatomic air, the
  same convention as ``nearspace.atmosphere``).
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple, Union

import numpy as np

try:  # keep the module usable both as a package member and as a script
    from .atmosphere import GAMMA_AIR, K_B, R_AIR
except ImportError:  # pragma: no cover - direct execution
    from atmosphere import GAMMA_AIR, K_B, R_AIR  # type: ignore

try:  # pragma: no cover - exercised implicitly
    import pymsis
except Exception:  # pragma: no cover - optional dependency
    pymsis = None  # type: ignore[assignment]

__all__ = [
    "RealAtmosphere",
    "MSIS2Atmosphere",
    "US76Atmosphere",
    "compare_backends",
    "DEFAULT_CACHE_DIR",
]

ArrayLike = Union[float, Sequence[float], np.ndarray]

#: Default location of the MSIS2 lookup-table cache (``<repo>/results``).
DEFAULT_CACHE_DIR = Path(__file__).resolve().parent.parent / "results"

#: Default epoch of the NRLMSIS 2.0 evaluation.
DEFAULT_DATE = datetime(2024, 1, 1, 0)

_MSIS2_ALIASES = {"msis2", "msis", "msis-2", "nrlmsis2", "nrlmsis-2.0", "nrlmsis"}
_US76_ALIASES = {"us76", "ussa76", "us-1976", "us1976", "us_standard_1976"}


# ==========================================================================
# U.S. Standard Atmosphere 1976
# ==========================================================================
_US76_R0 = 6_356_766.0          # m, nominal Earth radius used by the standard
_US76_G0 = 9.80665              # m/s^2, standard gravity
_US76_RSTAR = 8314.32           # J/(kmol K), universal gas constant
_US76_M0 = 28.9644              # kg/kmol, sea-level mean molecular weight
_US76_R_AIR = _US76_RSTAR / _US76_M0   # 287.05287 J/(kg K)
_US76_P0 = 101_325.0            # Pa
_US76_T0 = 288.15               # K
_US76_Z_TOP_LOWER = 86_000.0    # m geometric; upper model base
_US76_Z_MAX = 1_000_000.0       # m geometric; model ceiling

#: Seven lower layers: (geopotential base H [m], base T [K], dT/dH [K/m]).
_US76_LAYERS: Tuple[Tuple[float, float, float], ...] = (
    (0.0, 288.15, -0.0065),
    (11_000.0, 216.65, 0.0),
    (20_000.0, 216.65, 0.001),
    (32_000.0, 228.65, 0.0028),
    (47_000.0, 270.65, 0.0),
    (51_000.0, 270.65, -0.0028),
    (71_000.0, 214.65, -0.002),
)
#: Geopotential altitude of each layer top; H = 84852 m <-> z = 86 km.
_US76_LAYER_HTOP = np.array([11_000.0, 20_000.0, 32_000.0, 47_000.0,
                             51_000.0, 71_000.0, 84_852.0])

#: Mean molecular weight breakpoints of the thermosphere (US76 composition).
_US76_UPPER_M_Z_KM = np.array([86.0, 90.0, 95.0, 100.0, 110.0, 120.0,
                               150.0, 200.0, 300.0, 500.0, 1000.0])
_US76_UPPER_M = np.array([28.9644, 28.910, 28.740, 28.390, 27.270, 26.190,
                          24.100, 21.300, 17.720, 14.330, 3.940])


def _geopotential(z_m: np.ndarray) -> np.ndarray:
    """Geometric altitude [m] -> geopotential altitude [m]."""
    return _US76_R0 * z_m / (_US76_R0 + z_m)


def _us76_layer_anchors() -> Tuple[Tuple[float, float, float], ...]:
    """Base (H, T, p) of every lower layer, integrated hydrostatically."""
    anchors = [(0.0, _US76_T0, _US76_P0)]
    p = _US76_P0
    for (h_lo, t_lo, lapse), h_hi in zip(_US76_LAYERS, _US76_LAYER_HTOP):
        if lapse == 0.0:
            p = p * np.exp(-_US76_G0 * (h_hi - h_lo) / (_US76_R_AIR * t_lo))
        else:
            t_hi = t_lo + lapse * (h_hi - h_lo)
            p = p * (t_hi / t_lo) ** (-_US76_G0 / (_US76_R_AIR * lapse))
        anchors.append((h_hi, t_lo + lapse * (h_hi - h_lo), p))
    return tuple(anchors)


_US76_ANCHORS = _us76_layer_anchors()        # 8 entries: layer bases + 86 km top
_US76_LAYER_BASE = _US76_ANCHORS[:-1]        # (H, T, p) at each of the 7 layer bases
_US76_ANCHOR_H = np.array([a[0] for a in _US76_LAYER_BASE])
_US76_ANCHOR_T = np.array([a[1] for a in _US76_LAYER_BASE])
_US76_ANCHOR_P = np.array([a[2] for a in _US76_LAYER_BASE])
_US76_ANCHOR_L = np.array([lay[2] for lay in _US76_LAYERS])
_US76_P_86KM = float(_US76_ANCHORS[-1][2])   # hydrostatic anchor of the thermosphere


def _us76_temperature_upper(z_m: np.ndarray) -> np.ndarray:
    """Molecular temperature [K] of the US76 thermosphere (86-1000+ km)."""
    zk = np.asarray(z_m, dtype=float) / 1000.0
    r0k = _US76_R0 / 1000.0
    seg_iso = np.full_like(zk, 186.8673)
    arg = 1.0 - ((zk - 91.0) / 19.9429) ** 2
    seg_sqrt = 263.1905 - 76.3232 * np.sqrt(np.maximum(arg, 0.0))
    seg_lin = 240.0 + 12.0 * (zk - 110.0)
    eps = (zk - 120.0) * (r0k + 120.0) / (r0k + zk)
    seg_exp = 1000.0 - 640.0 * np.exp(-0.01875 * eps)
    return np.where(zk <= 91.0, seg_iso,
                    np.where(zk <= 110.0, seg_sqrt,
                             np.where(zk <= 120.0, seg_lin, seg_exp)))


class US76Atmosphere:
    """U.S. Standard Atmosphere 1976, analytic layered model (0-1000 km).

    The lower atmosphere (0-86 km geometric) uses the seven standard
    temperature-gradient layers written in geopotential altitude; the
    thermosphere (86-1000 km) uses the standard's analytic molecular
    temperature branches together with the tabulated mean molecular weight and
    a hydrostatic integration.  Kinetic temperature equals the molecular
    temperature below 86 km.
    """

    def __init__(self, z_step_upper: float = 100.0,
                 z_top_upper: float = _US76_Z_MAX) -> None:
        """Pre-integrate the thermosphere pressure profile.

        Parameters
        ----------
        z_step_upper:
            Integration step [m] of the thermosphere hydrostatic integration.
        z_top_upper:
            Upper limit [m] of the stored thermosphere table.
        """
        self.h_min = 0.0
        self.h_max = float(z_top_upper)
        self._z_step_upper = float(z_step_upper)
        self._z_up, self._log_p_up, self._log_rho_up = self._build_upper()

    # -- construction ------------------------------------------------------
    def _build_upper(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Integrate ``dp/dz = -p M g /(R* T)`` above 86 km."""
        z = np.arange(_US76_Z_TOP_LOWER, self.h_max + 0.5 * self._z_step_upper,
                      self._z_step_upper)
        if z[-1] < self.h_max:
            z = np.append(z, self.h_max)
        T = _us76_temperature_upper(z)
        M = np.interp(z / 1000.0, _US76_UPPER_M_Z_KM, _US76_UPPER_M)
        g = _US76_G0 * (_US76_R0 / (_US76_R0 + z)) ** 2
        # separable ODE: d(ln p)/dz = -M g /(R* T)
        c = M * g / (_US76_RSTAR * T)
        dz = np.diff(z)
        log_p = np.empty_like(z)
        log_p[0] = np.log(_US76_P_86KM)
        log_p[1:] = log_p[0] - np.cumsum(0.5 * (c[1:] + c[:-1]) * dz)
        log_rho = log_p + np.log(M) - np.log(_US76_RSTAR * T)
        return z, log_p, log_rho

    # -- analytic lower atmosphere ----------------------------------------
    @staticmethod
    def _lower(z_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return ``(T, p, rho)`` of the 0-86 km layer stack."""
        H = _geopotential(z_m)
        idx = np.clip(np.searchsorted(_US76_LAYER_HTOP, H, side="left"), 0, 6)
        h_lo = _US76_ANCHOR_H[idx]
        t_lo = _US76_ANCHOR_T[idx]
        p_lo = _US76_ANCHOR_P[idx]
        lapse = _US76_ANCHOR_L[idx]
        T = t_lo + lapse * (H - h_lo)
        p_iso = p_lo * np.exp(-_US76_G0 * (H - h_lo) / (_US76_R_AIR * t_lo))
        lapse_safe = np.where(lapse == 0.0, 1.0, lapse)
        p_lap = p_lo * (T / t_lo) ** (-_US76_G0 / (_US76_R_AIR * lapse_safe))
        p = np.where(lapse == 0.0, p_iso, p_lap)
        return T, p, p / (_US76_R_AIR * T)

    @staticmethod
    def _clip(h: ArrayLike) -> np.ndarray:
        return np.clip(np.asarray(h, dtype=float), 0.0, _US76_Z_MAX)

    def _lower_safe(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``_lower`` evaluated only inside its 0-86 km validity domain.

        Values above 86 km are replaced by the 86 km value; they are discarded
        by the ``np.where`` that combines the lower and thermosphere branches.
        """
        return self._lower(np.minimum(z, _US76_Z_TOP_LOWER))

    # -- interface ---------------------------------------------------------
    def T_array(self, h: ArrayLike) -> np.ndarray:
        """Temperature [K]."""
        z = self._clip(h)
        t_lo = self._lower_safe(z)[0]
        t_up = _us76_temperature_upper(np.maximum(z, _US76_Z_TOP_LOWER))
        return np.where(z <= _US76_Z_TOP_LOWER, t_lo, t_up)

    def p_array(self, h: ArrayLike) -> np.ndarray:
        """Static pressure [Pa]."""
        z = self._clip(h)
        p_lo = self._lower_safe(z)[1]
        p_up = np.exp(np.interp(z, self._z_up, self._log_p_up))
        return np.where(z <= _US76_Z_TOP_LOWER, p_lo, p_up)

    def rho_array(self, h: ArrayLike) -> np.ndarray:
        """Mass density [kg/m^3]."""
        z = self._clip(h)
        rho_lo = self._lower_safe(z)[2]
        rho_up = np.exp(np.interp(z, self._z_up, self._log_rho_up))
        return np.where(z <= _US76_Z_TOP_LOWER, rho_lo, rho_up)

    def a_array(self, h: ArrayLike) -> np.ndarray:
        """Speed of sound [m/s]."""
        return np.sqrt(GAMMA_AIR * self.p_array(h) / self.rho_array(h))

    def mean_free_path_array(self, h: ArrayLike,
                             d_molecule: float = 3.7e-10) -> np.ndarray:
        """Mean free path [m] for a hard-sphere diameter ``d_molecule``."""
        return (K_B * self.T_array(h)
                / (np.sqrt(2.0) * np.pi * d_molecule ** 2 * self.p_array(h)))

    def knudsen_array(self, h: ArrayLike, cbar: float) -> np.ndarray:
        """Knudsen number ``lambda / cbar``."""
        return self.mean_free_path_array(h) / float(cbar)

    def rho(self, h: float) -> float:
        """Mass density [kg/m^3] at geometric altitude ``h`` [m]."""
        return float(self.rho_array(h))

    def T(self, h: float) -> float:
        """Temperature [K]."""
        return float(self.T_array(h))

    def a(self, h: float) -> float:
        """Speed of sound [m/s]."""
        return float(self.a_array(h))

    def p(self, h: float) -> float:
        """Static pressure [Pa]."""
        return float(self.p_array(h))

    def mean_free_path(self, h: float, d_molecule: float = 3.7e-10) -> float:
        """Mean free path [m]."""
        return float(self.mean_free_path_array(h, d_molecule))

    def knudsen(self, h: float, cbar: float) -> float:
        """Knudsen number [-]."""
        return float(self.knudsen_array(h, cbar))

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return (f"US76Atmosphere(0-{self.h_max / 1000.0:.0f} km, "
                f"upper step {self._z_step_upper:g} m)")


# ==========================================================================
# NRLMSIS 2.0 (pymsis) with on-disk lookup table
# ==========================================================================
class MSIS2Atmosphere:
    """NRLMSIS 2.0 atmosphere sampled onto a cached altitude grid.

    The model is evaluated once with :func:`pymsis.calculate` on a uniform
    altitude grid and the result is cached as ``results/msis2_table_<f107>.csv``.
    Subsequent instances reload the CSV when every constructor parameter
    matches, otherwise the table is recomputed.

    Queries outside ``[z_min_km, z_max_km]`` are clipped to the table range,
    matching the clipping behaviour of ``nearspace.atmosphere.Atmosphere``.
    """

    def __init__(self,
                 f107: float = 150.0,
                 f107a: float = 150.0,
                 ap: float = 4.0,
                 lon: float = 0.0,
                 lat: float = 45.0,
                 date: Optional[datetime] = None,
                 version: Union[float, str] = 2.0,
                 z_min_km: float = 0.0,
                 z_max_km: float = 120.0,
                 dz_km: float = 0.25,
                 cache_dir: Optional[Union[str, Path]] = None,
                 use_cache: bool = True,
                 force_recompute: bool = False) -> None:
        """Build (or load) the NRLMSIS 2.0 lookup table.

        Parameters
        ----------
        f107, f107a:
            Daily and 81-day mean F10.7 solar radio flux [sfu].
        ap:
            Ap geomagnetic index, replicated into the seven 3-hourly slots.
        lon, lat:
            Geographic longitude/latitude [deg] of the evaluation point.
        date:
            UTC epoch of the evaluation (default 2024-01-01T00:00).
        version:
            NRLMSIS version passed to pymsis (default 2.0).
        z_min_km, z_max_km, dz_km:
            Altitude grid of the table.
        cache_dir:
            Directory of the CSV cache (default :data:`DEFAULT_CACHE_DIR`).
        use_cache:
            Read/write the CSV cache.
        force_recompute:
            Ignore an existing cache and re-evaluate NRLMSIS.
        """
        if pymsis is None:  # pragma: no cover - environment dependent
            raise ImportError("pymsis is required for backend='msis2'")

        self.f107 = float(f107)
        self.f107a = float(f107a)
        self.ap = float(ap)
        self.lon = float(lon)
        self.lat = float(lat)
        self.date = DEFAULT_DATE if date is None else date
        self.version = version
        self.z_min_km = float(z_min_km)
        self.z_max_km = float(z_max_km)
        self.dz_km = float(dz_km)
        self.use_cache = bool(use_cache)
        self.cache_path = (Path(cache_dir) if cache_dir is not None
                           else DEFAULT_CACHE_DIR) / f"msis2_table_{self.f107:g}.csv"

        self.h_min = self.z_min_km * 1000.0
        self.h_max = self.z_max_km * 1000.0

        self._h: np.ndarray = np.empty(0)
        self._log_rho: np.ndarray = np.empty(0)
        self._log_p: np.ndarray = np.empty(0)
        self._T: np.ndarray = np.empty(0)

        if self.use_cache and not force_recompute and self._load_cache():
            self.loaded_from_cache = True
        else:
            self._compute_table()
            if self.use_cache:
                self._save_cache()
            self.loaded_from_cache = False

    # -- model evaluation --------------------------------------------------
    def _grid_m(self) -> np.ndarray:
        n = int(round((self.z_max_km - self.z_min_km) / self.dz_km)) + 1
        return self.z_min_km * 1000.0 + np.arange(n, dtype=float) * self.dz_km * 1000.0

    def _evaluate(self, z_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate NRLMSIS 2.0 at ``z_m`` and return ``(rho, T, p)``."""
        aps = np.full((1, 7), self.ap, dtype=float)
        raw = pymsis.calculate(self.date, self.lon, self.lat, z_m / 1000.0,
                               f107s=self.f107, f107as=self.f107a, aps=aps,
                               version=self.version)
        arr = np.asarray(raw, dtype=float)
        if arr.ndim == 5:                       # (ndate, nlon, nlat, nalt, nvar)
            series = arr[0, 0, 0]
        elif arr.ndim == 2:                     # squeezed single-altitude call
            series = arr
        elif arr.ndim == 1:
            series = arr[None, :]
        else:                                   # pragma: no cover - defensive
            raise ValueError(f"unexpected pymsis output shape {arr.shape}")

        rho = series[:, pymsis.Variable.MASS_DENSITY]
        T = series[:, pymsis.Variable.TEMPERATURE]
        # static pressure from the composition: p = n_total * k_B * T.
        species = [v for v in pymsis.Variable
                   if v not in (pymsis.Variable.MASS_DENSITY,
                                pymsis.Variable.TEMPERATURE)]
        n_tot = np.nansum(series[:, species], axis=1)
        p = n_tot * K_B * T
        p = np.where(np.isfinite(p) & (p > 0.0), p, rho * R_AIR * T)
        return rho, T, p

    def _compute_table(self) -> None:
        """Fill the in-memory lookup table from a fresh NRLMSIS evaluation."""
        z = self._grid_m()
        rho, T, p = self._evaluate(z)
        self._h = z
        self._log_rho = np.log(rho)
        self._log_p = np.log(p)
        self._T = T

    # -- cache -------------------------------------------------------------
    def _metadata(self) -> Dict[str, str]:
        date = self.date.isoformat() if hasattr(self.date, "isoformat") else str(self.date)
        return {
            "engine": "pymsis-nrlmsis2",
            "version": f"{self.version:g}" if isinstance(self.version, (int, float))
                       else str(self.version),
            "f107": f"{self.f107:g}",
            "f107a": f"{self.f107a:g}",
            "ap": f"{self.ap:g}",
            "lon": f"{self.lon:g}",
            "lat": f"{self.lat:g}",
            "date": date,
            "z_min_km": f"{self.z_min_km:g}",
            "z_max_km": f"{self.z_max_km:g}",
            "dz_km": f"{self.dz_km:g}",
            "n": str(int(self._grid_m().size)),
        }

    def _save_cache(self) -> None:
        """Write the table (metadata header + CSV rows) to ``cache_path``."""
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        with self.cache_path.open("w", encoding="utf-8", newline="") as fh:
            fh.write("# NRLMSIS 2.0 atmosphere lookup table "
                     "(log-linear interpolation, see nearspace/atmosphere_real.py)\n")
            for key, val in self._metadata().items():
                fh.write(f"# {key}={val}\n")
            fh.write("h_m,rho_kg_m3,T_K,p_Pa\n")
            for i in range(self._h.size):
                fh.write("%.6f,%.10e,%.10e,%.10e\n"
                         % (self._h[i], float(np.exp(self._log_rho[i])),
                            self._T[i], float(np.exp(self._log_p[i]))))

    def _load_cache(self) -> bool:
        """Load ``cache_path`` if it exists and matches the current settings."""
        if not self.cache_path.exists():
            return False
        meta: Dict[str, str] = {}
        rows: list = []
        header_seen = False
        try:
            with self.cache_path.open("r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        body = line.lstrip("#").strip()
                        if "=" in body:
                            key, val = body.split("=", 1)
                            meta[key.strip()] = val.strip()
                        continue
                    if not header_seen:          # column names row
                        header_seen = True
                        continue
                    rows.append([float(c) for c in line.split(",")])
        except Exception:                        # pragma: no cover - corrupt cache
            return False
        expected = self._metadata()
        if any(meta.get(k) != v for k, v in expected.items()):
            return False
        data = np.asarray(rows, dtype=float)
        if data.ndim != 2 or data.shape[1] != 4 or data.shape[0] != int(expected["n"]):
            return False
        self._h = data[:, 0]
        self._log_rho = np.log(data[:, 1])
        self._T = data[:, 2]
        self._log_p = np.log(data[:, 3])
        return True

    def rerun_for_conditions(self, f107: float, f107a: Optional[float] = None,
                             ap: Optional[float] = None,
                             force_recompute: bool = False) -> "MSIS2Atmosphere":
        """Re-evaluate the table for a new solar/geomagnetic condition set.

        Used for solar-activity scans; the internal table, the cache path and
        (optionally) a matching CSV cache are refreshed in place.
        """
        self.f107 = float(f107)
        if f107a is not None:
            self.f107a = float(f107a)
        if ap is not None:
            self.ap = float(ap)
        self.cache_path = (self.cache_path.parent
                           / f"msis2_table_{self.f107:g}.csv")
        if not (self.use_cache and not force_recompute and self._load_cache()):
            self._compute_table()
            if self.use_cache:
                self._save_cache()
            self.loaded_from_cache = False
        else:
            self.loaded_from_cache = True
        return self

    # -- interface ---------------------------------------------------------
    @staticmethod
    def _as_array(h: ArrayLike) -> np.ndarray:
        return np.asarray(h, dtype=float)

    def rho_array(self, h: ArrayLike) -> np.ndarray:
        """Mass density [kg/m^3]."""
        return np.exp(np.interp(self._as_array(h), self._h, self._log_rho))

    def T_array(self, h: ArrayLike) -> np.ndarray:
        """Temperature [K]."""
        return np.interp(self._as_array(h), self._h, self._T)

    def p_array(self, h: ArrayLike) -> np.ndarray:
        """Static pressure [Pa]."""
        return np.exp(np.interp(self._as_array(h), self._h, self._log_p))

    def a_array(self, h: ArrayLike) -> np.ndarray:
        """Speed of sound [m/s]."""
        return np.sqrt(GAMMA_AIR * self.p_array(h) / self.rho_array(h))

    def mean_free_path_array(self, h: ArrayLike,
                             d_molecule: float = 3.7e-10) -> np.ndarray:
        """Mean free path [m] for a hard-sphere diameter ``d_molecule``."""
        return (K_B * self.T_array(h)
                / (np.sqrt(2.0) * np.pi * d_molecule ** 2 * self.p_array(h)))

    def knudsen_array(self, h: ArrayLike, cbar: float) -> np.ndarray:
        """Knudsen number ``lambda / cbar``."""
        return self.mean_free_path_array(h) / float(cbar)

    def rho(self, h: float) -> float:
        """Mass density [kg/m^3] at geometric altitude ``h`` [m]."""
        return float(self.rho_array(h))

    def T(self, h: float) -> float:
        """Temperature [K]."""
        return float(self.T_array(h))

    def a(self, h: float) -> float:
        """Speed of sound [m/s]."""
        return float(self.a_array(h))

    def p(self, h: float) -> float:
        """Static pressure [Pa]."""
        return float(self.p_array(h))

    def mean_free_path(self, h: float, d_molecule: float = 3.7e-10) -> float:
        """Mean free path [m]."""
        return float(self.mean_free_path_array(h, d_molecule))

    def knudsen(self, h: float, cbar: float) -> float:
        """Knudsen number [-]."""
        return float(self.knudsen_array(h, cbar))

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        src = "cache" if self.loaded_from_cache else "pymsis"
        return (f"MSIS2Atmosphere(F10.7={self.f107:g}, Ap={self.ap:g}, "
                f"lat={self.lat:g}, lon={self.lon:g}, "
                f"{self._h.size} pts, {src})")


# ==========================================================================
# Facade
# ==========================================================================
class RealAtmosphere:
    """Backend-agnostic real atmosphere; drop-in for ``Atmosphere``.

    Examples
    --------
    >>> atm = RealAtmosphere("msis2", f107=150.0, ap=4.0)   # NRLMSIS 2.0
    >>> atm = RealAtmosphere("us76")                        # US Standard 1976
    >>> atm.rho(100_000.0)                                  # kg/m^3
    """

    def __init__(self, backend: str = "msis2", **kwargs: Any) -> None:
        key = str(backend).strip().lower()
        if key in _MSIS2_ALIASES:
            self._impl: Union[MSIS2Atmosphere, US76Atmosphere] = MSIS2Atmosphere(**kwargs)
        elif key in _US76_ALIASES:
            self._impl = US76Atmosphere(**kwargs)
        elif key in ("old", "legacy", "exp", "piecewise"):
            raise ValueError(
                "the piecewise-exponential backend lives in "
                "nearspace.atmosphere.Atmosphere; use that class directly")
        else:
            raise ValueError(
                f"unknown backend {backend!r}; expected 'msis2' or 'us76'")
        self.backend = key

    # -- delegated properties ---------------------------------------------
    @property
    def h_min(self) -> float:
        """Lowest representable geometric altitude [m]."""
        return self._impl.h_min

    @property
    def h_max(self) -> float:
        """Highest representable geometric altitude [m]."""
        return self._impl.h_max

    @property
    def impl(self) -> Union[MSIS2Atmosphere, US76Atmosphere]:
        """The underlying backend instance."""
        return self._impl

    @property
    def loaded_from_cache(self) -> bool:
        """``True`` when the MSIS2 backend was restored from its CSV cache."""
        return bool(getattr(self._impl, "loaded_from_cache", False))

    @property
    def cache_path(self) -> Optional[Path]:
        """CSV cache path of the MSIS2 backend (``None`` for US76)."""
        return getattr(self._impl, "cache_path", None)

    def rerun_for_conditions(self, f107: float, f107a: Optional[float] = None,
                             ap: Optional[float] = None,
                             force_recompute: bool = False) -> "RealAtmosphere":
        """Forward :meth:`MSIS2Atmosphere.rerun_for_conditions` (MSIS2 only)."""
        if not hasattr(self._impl, "rerun_for_conditions"):
            raise AttributeError(
                "rerun_for_conditions is only available for backend='msis2'")
        self._impl.rerun_for_conditions(f107, f107a, ap, force_recompute)  # type: ignore[attr-defined]
        return self

    # -- scalar interface (identical to nearspace.atmosphere.Atmosphere) ---
    def rho(self, h: float) -> float:
        """Mass density [kg/m^3]."""
        return self._impl.rho(h)

    def T(self, h: float) -> float:
        """Temperature [K]."""
        return self._impl.T(h)

    def a(self, h: float) -> float:
        """Speed of sound [m/s]."""
        return self._impl.a(h)

    def p(self, h: float) -> float:
        """Static pressure [Pa]."""
        return self._impl.p(h)

    def mean_free_path(self, h: float, d_molecule: float = 3.7e-10) -> float:
        """Mean free path [m]."""
        return self._impl.mean_free_path(h, d_molecule)

    def knudsen(self, h: float, cbar: float) -> float:
        """Knudsen number [-]."""
        return self._impl.knudsen(h, cbar)

    # -- vectorised interface ---------------------------------------------
    def rho_array(self, h: ArrayLike) -> np.ndarray:
        """Vectorised :meth:`rho`."""
        return self._impl.rho_array(h)

    def T_array(self, h: ArrayLike) -> np.ndarray:
        """Vectorised :meth:`T`."""
        return self._impl.T_array(h)

    def a_array(self, h: ArrayLike) -> np.ndarray:
        """Vectorised :meth:`a`."""
        return self._impl.a_array(h)

    def p_array(self, h: ArrayLike) -> np.ndarray:
        """Vectorised :meth:`p`."""
        return self._impl.p_array(h)

    def mean_free_path_array(self, h: ArrayLike,
                             d_molecule: float = 3.7e-10) -> np.ndarray:
        """Vectorised :meth:`mean_free_path`."""
        return self._impl.mean_free_path_array(h, d_molecule)

    def knudsen_array(self, h: ArrayLike, cbar: float) -> np.ndarray:
        """Vectorised :meth:`knudsen`."""
        return self._impl.knudsen_array(h, cbar)

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return f"RealAtmosphere(backend={self.backend!r}, impl={self._impl!r})"


# ==========================================================================
# Comparison helper
# ==========================================================================
def compare_backends(alts_km: ArrayLike,
                     us76_kwargs: Optional[Dict[str, Any]] = None,
                     cbar: float = 3.2,
                     **msis2_kwargs: Any) -> Dict[str, np.ndarray]:
    """Tabulate the MSIS2 and US76 backends side by side.

    Parameters
    ----------
    alts_km:
        Geometric altitudes [km] at which both models are evaluated.
    us76_kwargs:
        Optional keyword arguments for the US76 backend.
    cbar:
        Mean aerodynamic chord [m] used for the Knudsen number column.
    **msis2_kwargs:
        Keyword arguments for the MSIS2 backend (``f107``, ``ap``, ``lat``,
        ``date``, ``force_recompute``, ...).

    Returns
    -------
    dict of numpy.ndarray
        ``alt_km``, ``rho_msis2``, ``T_msis2``, ``a_msis2``, ``p_msis2``,
        ``mfp_msis2``, ``Kn_msis2``, ``rho_us76``, ``T_us76``, ``a_us76``,
        ``p_us76``, ``rho_ratio`` (MSIS2/US76), ``T_diff_K`` (MSIS2-US76).
    """
    alt_km = np.asarray(alts_km, dtype=float)
    h = alt_km * 1000.0
    msis = RealAtmosphere("msis2", **msis2_kwargs)
    us76 = RealAtmosphere("us76", **(us76_kwargs or {}))
    out: Dict[str, np.ndarray] = {"alt_km": alt_km}
    for tag, atm in (("msis2", msis), ("us76", us76)):
        out[f"rho_{tag}"] = atm.rho_array(h)
        out[f"T_{tag}"] = atm.T_array(h)
        out[f"a_{tag}"] = atm.a_array(h)
        out[f"p_{tag}"] = atm.p_array(h)
    out["mfp_msis2"] = msis.mean_free_path_array(h)
    out["Kn_msis2"] = msis.knudsen_array(h, cbar)
    out["rho_ratio"] = out["rho_msis2"] / out["rho_us76"]
    out["T_diff_K"] = out["T_msis2"] - out["T_us76"]
    return out
