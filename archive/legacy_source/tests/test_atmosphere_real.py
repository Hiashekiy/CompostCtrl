"""Standalone verification of :mod:`nearspace.atmosphere_real`.

Run from the workspace root (no pytest required)::

    python tests/test_atmosphere_real.py

Checks
------
1. US76 backend against recognised U.S. Standard Atmosphere 1976 values.
2. NRLMSIS 2.0 (F10.7 = 150, Ap = 4) profile, monotonicity and magnitudes.
3. Solar-activity sensitivity (F10.7 = 70 / 150 / 250 at 100 km).
4. Deviation from the legacy 8-layer exponential ``nearspace.atmosphere``.
5. Lookup-table performance (12 000 altitude queries) and CSV caching.
6. Speed of sound, pressure, mean free path and Knudsen number sanity.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from nearspace.atmosphere import Atmosphere                       # noqa: E402
from nearspace.atmosphere_real import (RealAtmosphere, US76Atmosphere,  # noqa: E402
                                       compare_backends)

MASS_CHORD = 3.2          # m, cbar from config.yaml
_FAILURES: list[str] = []


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def check(cond: bool, msg: str) -> None:
    """Print a PASS/FAIL line and remember failures."""
    print(f"      [{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond:
        _FAILURES.append(msg)


def banner(title: str) -> None:
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def rel_err(value: float, ref: float) -> float:
    return 100.0 * (value / ref - 1.0)


def legacy_atmosphere() -> Atmosphere:
    """Build the legacy 8-layer exponential model from config.yaml."""
    with open(ROOT / "config.yaml", "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    layers = [SimpleNamespace(**d) for d in cfg["environment"]["layers"]]
    return Atmosphere(layers)


# --------------------------------------------------------------------------
# 1. US76 vs recognised standard values
# --------------------------------------------------------------------------
#: Recognised US Standard 1976 values: z[km] -> (T [K], rho [kg/m^3]).
US76_REFERENCE = {
    20.0: (216.65, 8.8910e-02),
    30.0: (226.51, 1.8410e-02),
    40.0: (250.35, 3.9957e-03),
    50.0: (270.65, 1.0269e-03),
    60.0: (247.02, 3.0968e-04),
    70.0: (219.58, 8.2829e-05),
    80.0: (198.64, 1.8458e-05),
    86.0: (186.87, 6.9582e-06),
    100.0: (195.08, 5.6040e-07),
}


def test_us76_reference() -> US76Atmosphere:
    banner("[1] US76 backend vs. recognised U.S. Standard Atmosphere 1976 values")
    atm = US76Atmosphere()
    print(f"      model: {atm!r}")
    print(f"      {'z[km]':>6} {'T[K]':>10} {'T_ref':>10} {'errT[%]':>8} "
          f"{'rho[kg/m^3]':>13} {'rho_ref':>13} {'errRho[%]':>9} {'p[Pa]':>11}")
    worst_t = worst_r = 0.0
    for z_km in sorted(US76_REFERENCE):
        t_ref, rho_ref = US76_REFERENCE[z_km]
        T = atm.T(z_km * 1e3)
        rho = atm.rho(z_km * 1e3)
        et, er = rel_err(T, t_ref), rel_err(rho, rho_ref)
        print(f"      {z_km:6.1f} {T:10.3f} {t_ref:10.2f} {et:+8.3f} "
              f"{rho:13.6e} {rho_ref:13.6e} {er:+9.3f} {atm.p(z_km * 1e3):11.4e}")
        if z_km <= 86.0:                       # analytic lower-atmosphere layers
            worst_t = max(worst_t, abs(et))
            worst_r = max(worst_r, abs(er))
    print(f"      worst |err| for z <= 86 km: T = {worst_t:.4f} %, "
          f"rho = {worst_r:.4f} %")
    check(worst_r < 1.0, "US76 density within 1 % of the standard below 86 km")
    check(worst_t < 1.0, "US76 temperature within 1 % of the standard below 86 km")
    # The 86-1000 km part is the standard's analytic approximation.
    t100, r100 = US76_REFERENCE[100.0]
    check(abs(rel_err(atm.T(100e3), t100)) < 1.0,
          "US76 thermosphere T(100 km) within 1 %")
    check(abs(rel_err(atm.rho(100e3), r100)) < 3.0,
          "US76 thermosphere rho(100 km) within 3 % (analytic approximation)")
    # Monotonic decrease of density over the whole lower atmosphere.
    hs = np.linspace(0.0, 86e3, 2000)
    check(bool(np.all(np.diff(atm.rho_array(hs)) < 0.0)),
          "US76 density strictly decreasing on 0-86 km")
    return atm


# --------------------------------------------------------------------------
# 2. MSIS2 profile 20-120 km
# --------------------------------------------------------------------------
def test_msis2_profile() -> RealAtmosphere:
    banner("[2] NRLMSIS 2.0 backend (f107 = f107a = 150, Ap = 4, lat = 45 deg, lon = 0)")
    atm = RealAtmosphere("msis2", f107=150.0, f107a=150.0, ap=4.0,
                         force_recompute=True)
    print(f"      model: {atm!r}")
    print(f"      table: {atm.cache_path}")
    print(f"      {'z[km]':>6} {'rho[kg/m^3]':>13} {'T[K]':>9} {'a[m/s]':>8} "
          f"{'p[Pa]':>11} {'mfp[m]':>10} {'Kn(cbar=3.2)':>13}")
    for z_km in [20, 30, 40, 50, 60, 70, 80, 86, 90, 100, 110, 120]:
        z = z_km * 1e3
        print(f"      {z_km:6.1f} {atm.rho(z):13.6e} {atm.T(z):9.3f} "
              f"{atm.a(z):8.2f} {atm.p(z):11.4e} {atm.mean_free_path(z):10.3e} "
              f"{atm.knudsen(z, MASS_CHORD):13.4e}")

    hs = np.linspace(20e3, 120e3, 4001)
    rho = atm.rho_array(hs)
    check(bool(np.all(np.diff(rho) < 0.0)), "rho strictly decreasing on 20-120 km")
    check(bool(np.all(np.isfinite(rho))), "rho finite everywhere on 20-120 km")
    r100 = atm.rho(100e3)
    check(1e-8 <= r100 <= 1e-5,
          f"rho(100 km) = {r100:.4e} kg/m^3 inside [1e-8, 1e-5]")
    check(1e-6 <= atm.rho(80e3) <= 1e-4,
          f"rho(80 km) = {atm.rho(80e3):.4e} kg/m^3 of the expected order")
    check(150.0 <= atm.T(100e3) <= 260.0,
          f"T(100 km) = {atm.T(100e3):.2f} K of the expected order")

    # interpolation error of the 0.25 km lookup table vs. a direct MSIS2 call
    import pymsis
    from datetime import datetime
    mid = np.array([20.125, 45.375, 78.625, 99.875]) * 1e3
    direct = np.asarray(pymsis.calculate(
        datetime(2024, 1, 1, 0), 0.0, 45.0, mid / 1e3,
        f107s=150.0, f107as=150.0, aps=np.full((1, 7), 4.0), version=2.0))
    if direct.ndim == 5:
        direct = direct[0, 0, 0]
    rho_direct = direct[:, pymsis.Variable.MASS_DENSITY]
    err = np.max(np.abs(atm.rho_array(mid) / rho_direct - 1.0))
    print(f"      max lookup-table interpolation error (4 mid-grid points): {err:.3e}")
    check(err < 5e-3, "lookup-table interpolation error < 0.5 %")
    return atm


# --------------------------------------------------------------------------
# 3. solar activity sensitivity
# --------------------------------------------------------------------------
def test_solar_activity(atm: RealAtmosphere) -> None:
    banner("[3] Solar-activity sensitivity (F10.7 = 70 / 150 / 250, Ap = 4)")
    print(f"      {'F10.7[sfu]':>10} {'rho(100 km)[kg/m^3]':>21} {'T(100 km)[K]':>13} "
          f"{'rho(200 km)[kg/m^3]':>21}")
    rho100: dict[float, float] = {}
    rho200: dict[float, float] = {}
    for f107 in (70.0, 150.0, 250.0):
        atm.rerun_for_conditions(f107=f107, f107a=f107, ap=4.0)
        rho100[f107] = atm.rho(100e3)
        # a separate 250 km table is needed for the thermosphere column
        upper = RealAtmosphere("msis2", f107=f107, f107a=f107, ap=4.0,
                               z_max_km=250.0, use_cache=False)
        rho200[f107] = upper.rho(200e3)
        print(f"      {f107:10.1f} {rho100[f107]:21.6e} {atm.T(100e3):13.3f} "
              f"{rho200[f107]:21.6e}")
    ratio100 = rho100[250.0] / rho100[70.0]
    ratio200 = rho200[250.0] / rho200[70.0]
    print(f"      rho(250)/rho(70):  {ratio100:.3f} at 100 km, {ratio200:.3f} at 200 km")
    check(rho100[150.0] > rho100[70.0] and rho100[250.0] > rho100[150.0],
          "density at 100 km increases monotonically with F10.7")
    check(ratio100 > 1.02,
          f"F10.7 = 250 density exceeds F10.7 = 70 by {ratio100:.3f}x at 100 km")
    check(ratio200 > 2.0,
          f"F10.7 = 250 density exceeds F10.7 = 70 by more than 2x at 200 km "
          f"({ratio200:.2f}x)")
    print("      NOTE: NRLMSIS 2.0 is only weakly F10.7-sensitive near 100 km, where")
    print("            the atmosphere is still well mixed; the large solar-cycle")
    print("            contrast develops higher up (2.4x at 200 km, ~7x at 300 km).")
    # restore the nominal conditions for the remaining tests
    atm.rerun_for_conditions(f107=150.0, f107a=150.0, ap=4.0)


# --------------------------------------------------------------------------
# 4. deviation from the legacy 8-layer exponential model
# --------------------------------------------------------------------------
def test_against_legacy(msis: RealAtmosphere, us76: US76Atmosphere) -> None:
    banner("[4] Deviation from the legacy 8-layer exponential Atmosphere")
    legacy = legacy_atmosphere()
    print(f"      legacy layers: {legacy.h_min / 1e3:.0f}-{legacy.h_max / 1e3:.0f} km")
    print(f"      {'z[km]':>6} {'rho_legacy':>13} {'rho_msis2':>13} {'dev_msis2[%]':>13} "
          f"{'rho_us76':>13} {'dev_us76[%]':>12} {'T_msis2-T_legacy[K]':>20}")
    devs = []
    for z_km in [20, 30, 40, 50, 60, 70, 80, 86, 90, 100]:
        z = z_km * 1e3
        r_leg = legacy.rho(z)
        r_ms = msis.rho(z)
        r_us = us76.rho(z)
        dev_ms = rel_err(r_ms, r_leg)
        dev_us = rel_err(r_us, r_leg)
        devs.append(abs(dev_ms))
        print(f"      {z_km:6.1f} {r_leg:13.6e} {r_ms:13.6e} {dev_ms:+13.2f} "
              f"{r_us:13.6e} {dev_us:+12.2f} {msis.T(z) - legacy.T(z):20.2f}")
    print(f"      max |deviation| MSIS2 vs legacy on 20-100 km: {max(devs):.2f} %")
    check(all(np.isfinite(d) for d in devs), "all deviations finite")
    check(max(devs) < 100.0,
          "MSIS2-vs-legacy density deviation stays below 100 % on 20-100 km")
    check(abs(rel_err(legacy.rho(20e3), us76.rho(20e3))) < 2.0,
          "legacy fit is within 2 % of US76 at 20 km (fit sanity)")


# --------------------------------------------------------------------------
# 5. performance and caching
# --------------------------------------------------------------------------
def test_performance_and_cache(atm: RealAtmosphere) -> None:
    banner("[5] Lookup-table performance (12 000 altitude queries) and CSV cache")
    hs = np.linspace(20e3, 100e3, 12_000)
    t0 = time.perf_counter()
    rho_vec = atm.rho_array(hs)
    t_vec = time.perf_counter() - t0
    t0 = time.perf_counter()
    rho_loop = np.array([atm.rho(h) for h in hs])
    t_loop = time.perf_counter() - t0
    print(f"      rho_array(12 000 pts): {t_vec * 1e3:8.3f} ms")
    print(f"      scalar loop (12 000):  {t_loop * 1e3:8.3f} ms")
    print(f"      total:                 {(t_vec + t_loop) * 1e3:8.3f} ms")
    check(np.allclose(rho_vec, rho_loop, rtol=0.0, atol=0.0),
          "rho_array and the scalar loop agree bitwise")
    check(t_vec + t_loop < 1.0,
          f"12 000 density queries in {(t_vec + t_loop) * 1e3:.1f} ms < 1.0 s")

    cache = atm.cache_path
    check(cache is not None and Path(cache).exists(),
          f"cache file written: {cache}")
    t0 = time.perf_counter()
    atm2 = RealAtmosphere("msis2", f107=150.0, f107a=150.0, ap=4.0)
    t_load = time.perf_counter() - t0
    print(f"      reload from cache: {t_load * 1e3:.2f} ms "
          f"(loaded_from_cache={atm2.loaded_from_cache})")
    check(atm2.loaded_from_cache, "second instance restored from the CSV cache")
    check(bool(np.isclose(atm2.rho(100e3), atm.rho(100e3), rtol=1e-9)),
          "cached table reproduces the in-memory density to 1e-9 relative")
    with open(cache, "r", encoding="utf-8") as fh:
        head = [next(fh).rstrip() for _ in range(4)]
    print("      cache header:")
    for line in head:
        print(f"        {line}")
    # a different condition set must not silently reuse the stale table
    atm3 = RealAtmosphere("msis2", f107=150.0, f107a=150.0, ap=40.0)
    dev_ap = abs(rel_err(atm3.rho(100e3), atm.rho(100e3)))
    print(f"      |rho(100 km, Ap=40) / rho(100 km, Ap=4) - 1| = {dev_ap:.3f} %")
    check(dev_ap > 0.1,
          "changed Ap invalidates the cache and forces a re-evaluation")
    atm3.rerun_for_conditions(f107=150.0, f107a=150.0, ap=4.0)   # restore cache


# --------------------------------------------------------------------------
# 6. derived quantities
# --------------------------------------------------------------------------
def test_derived_quantities(msis: RealAtmosphere, us76: US76Atmosphere) -> None:
    banner("[6] Speed of sound, pressure, mean free path and Knudsen number")
    print(f"      {'model':>8} {'z[km]':>6} {'a[m/s]':>8} {'p[Pa]':>11} "
          f"{'mfp[m]':>10} {'Kn(cbar=3.2)':>13}")
    for name, atm in (("msis2", msis), ("us76", us76)):
        for z_km in (20, 60, 80, 100):
            z = z_km * 1e3
            print(f"      {name:>8} {z_km:6.1f} {atm.a(z):8.2f} {atm.p(z):11.4e} "
                  f"{atm.mean_free_path(z):10.3e} {atm.knudsen(z, MASS_CHORD):13.4e}")

    for name, atm in (("msis2", msis), ("us76", us76)):
        check(280.0 < atm.a(20e3) < 320.0,
              f"{name}: a(20 km) = {atm.a(20e3):.2f} m/s in 280-320 m/s")
        check(200.0 < atm.a(100e3) < 400.0,
              f"{name}: a(100 km) = {atm.a(100e3):.2f} m/s in 200-400 m/s")
        check(atm.p(100e3) > 0.0 and atm.p(100e3) < 0.1,
              f"{name}: p(100 km) = {atm.p(100e3):.4e} Pa inside (0, 0.1)")
        check(atm.mean_free_path(100e3) > 0.0,
              f"{name}: mean free path positive at 100 km")
        kn = atm.knudsen(100e3, MASS_CHORD)
        check(1e-3 < kn < 0.2,
              f"{name}: Kn(100 km, cbar=3.2 m) = {kn:.4e} inside (1e-3, 0.2)")

    # Kn must be the rarefaction driver: it grows by orders of magnitude
    kn20 = msis.knudsen(20e3, MASS_CHORD)
    kn100 = msis.knudsen(100e3, MASS_CHORD)
    check(kn100 / kn20 > 1e3,
          f"MSIS2 Kn grows from {kn20:.3e} (20 km) to {kn100:.3e} (100 km)")


# --------------------------------------------------------------------------
# 7. compare_backends helper
# --------------------------------------------------------------------------
def test_compare_backends() -> None:
    banner("[7] compare_backends() table helper")
    alts = np.array([20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 86.0, 100.0, 120.0])
    tab = compare_backends(alts, f107=150.0, ap=4.0, cbar=MASS_CHORD)
    print(f"      keys: {', '.join(sorted(tab))}")
    print(f"      {'z[km]':>6} {'rho_msis2':>13} {'rho_us76':>13} {'ratio':>7} "
          f"{'T_msis2':>9} {'T_us76':>9} {'dT[K]':>8} {'Kn':>10}")
    for i, z_km in enumerate(alts):
        print(f"      {z_km:6.1f} {tab['rho_msis2'][i]:13.6e} "
              f"{tab['rho_us76'][i]:13.6e} {tab['rho_ratio'][i]:7.3f} "
              f"{tab['T_msis2'][i]:9.2f} {tab['T_us76'][i]:9.2f} "
              f"{tab['T_diff_K'][i]:+8.2f} {tab['Kn_msis2'][i]:10.4e}")
    check(all(v.shape == alts.shape for k, v in tab.items() if k != "alt_km"),
          "all returned arrays have the requested shape")
    check(bool(np.all(np.isfinite(tab["rho_ratio"]))), "density ratios finite")
    check(bool(np.all(tab["rho_ratio"] < 1.0)),
          "MSIS2 is thinner than US76 over the whole 20-120 km band "
          "(solar-minimum-like F10.7 = 150)")


# --------------------------------------------------------------------------
def main() -> int:
    print("=" * 78)
    print("nearspace.atmosphere_real -- verification")
    print(f"workspace root : {ROOT}")
    print(f"numpy          : {np.__version__}")
    try:
        import pymsis
        print(f"pymsis         : {pymsis.__version__}")
    except Exception as exc:                      # pragma: no cover
        print(f"pymsis         : NOT AVAILABLE ({exc})")
    print("=" * 78)

    us76 = test_us76_reference()
    msis = test_msis2_profile()
    test_solar_activity(msis)
    test_against_legacy(msis, us76)
    test_performance_and_cache(msis)
    test_derived_quantities(msis, us76)
    test_compare_backends()

    banner("SUMMARY")
    if _FAILURES:
        print(f"      {len(_FAILURES)} check(s) FAILED:")
        for msg in _FAILURES:
            print(f"        - {msg}")
        print("\nRESULT: FAILED")
        return 1
    print("      all checks passed")
    print("\nRESULT: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
