"""Atmosphere backend factory (exp | us76 | msis2)."""
from __future__ import annotations

import os
from datetime import datetime


def make_atmosphere(cfg, cache_dir=None, backend=None):
    """Build the atmosphere object selected by cfg.environment.backend."""
    env = cfg.environment
    backend = (backend or getattr(env, "backend", "exp")).lower()
    if backend == "exp":
        from .atmosphere import Atmosphere
        return Atmosphere(env.layers)
    from .atmosphere_real import RealAtmosphere
    if backend == "us76":
        return RealAtmosphere("us76")
    if backend == "msis2":
        m = env.msis2
        return RealAtmosphere(
            "msis2", f107=float(m.f107), f107a=float(m.f107a), ap=float(m.ap),
            lat=float(m.lat), lon=float(m.lon),
            date=datetime.fromisoformat(str(m.date)),
            z_max_km=float(getattr(m, "z_max_km", 120.0)),
            dz_km=float(getattr(m, "dz_km", 0.25)),
            cache_dir=cache_dir,
        )
    raise ValueError("unknown atmosphere backend: %r" % backend)
