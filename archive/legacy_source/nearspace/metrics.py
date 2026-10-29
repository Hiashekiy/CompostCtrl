"""Metric definitions for the aero-surface / aero+CMG A/B comparison.

Every threshold below is FIXED BEFORE the runs (implementation plan §4.5 and
§5.5) and is copied into each run's metadata, so A and B can never be scored
with different tolerances.

Terminology
    err_norm_deg  quaternion geodesic error 2*acos|q_ref·q|, degrees
    err_*_deg     Euler (ZYX) difference between the realised attitude and the
                  reference attitude, degrees, wrapped to (-180, 180]
Both are recorded; every figure labels which one it uses.

Settling-time rules (fixed in advance, applied identically to A and B)
    disturbance recovery  error must fall below an ABSOLUTE band of
                          ``ERR_TOL_ABS_DEG`` AND below ``RECOVERY_REL_FRAC`` of
                          that run's own disturbance-window peak, and stay there
                          for ``HOLD_SEC``.  Both variants are reported; the
                          absolute one is the headline number.
    manoeuvre completion  error must fall below ``max(MAN_TOL_FLOOR_DEG,
                          MAN_TOL_FRAC*|delta_theta|)`` and stay there for
                          ``MAN_HOLD_SEC``.  A run that never enters the band is
                          reported as NOT COMPLETED (``None``), never as a
                          finite time.
"""
from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------------------
# FIXED SCORING CONSTANTS  (do not tune per case / per mode)
# ---------------------------------------------------------------------------
ERR_TOL_ABS_DEG = 0.05
RECOVERY_REL_FRAC = 0.10
HOLD_SEC = 2.0

MAN_TOL_FRAC = 0.05
MAN_TOL_FLOOR_DEG = 0.5
MAN_HOLD_SEC = 3.0

SAT_FRAC = 0.999
HOLD_WINDOW_SEC = 20.0
DIST_TAIL_SEC = 5.0

SCORING = dict(err_tol_abs_deg=ERR_TOL_ABS_DEG, recovery_rel_frac=RECOVERY_REL_FRAC,
               hold_sec=HOLD_SEC, man_tol_frac=MAN_TOL_FRAC,
               man_tol_floor_deg=MAN_TOL_FLOOR_DEG, man_hold_sec=MAN_HOLD_SEC,
               sat_frac=SAT_FRAC, hold_window_sec=HOLD_WINDOW_SEC,
               dist_tail_sec=DIST_TAIL_SEC)


def _hold_time(t, err, t_from, tol_deg, hold):
    """First time >= t_from at which err stays <= tol for `hold` seconds.

    Returns the *settling instant relative to t_from*, or None if the band is
    never held for long enough inside the record.
    """
    idx = int(np.searchsorted(t, t_from, side="left"))
    if idx >= len(t):
        return None
    inside = np.abs(err[idx:]) <= tol_deg
    if not inside.any():
        return None
    ti = t[idx:]
    run = 0
    for i in range(len(ti)):
        if inside[i]:
            run += 1
            if ti[i] - ti[i - run + 1] >= hold - 1e-9:
                return float(ti[i - run + 1] - t_from)
        else:
            run = 0
    return None


def _longest_run(flags):
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return int(best)






def compute(rec: dict, windows: dict | None = None, delta_max_deg=None) -> dict:
    """Compute every metric required by §4.5 / §5.5 from one run record."""
    windows = windows or {}
    t = np.asarray(rec["t"], dtype=float)
    en = np.asarray(rec["err_norm_deg"], dtype=float)
    ex = np.asarray(rec["err_roll_deg"], dtype=float)
    ey = np.asarray(rec["err_pitch_deg"], dtype=float)
    ez = np.asarray(rec["err_yaw_deg"], dtype=float)

    m: dict = {}
    m["t_end"] = float(t[-1])
    m["n_ctrl_steps"] = int(len(t))

    # -- 1/2/3 attitude error ------------------------------------------------
    m["err_rmse_deg"] = float(np.sqrt(np.mean(en ** 2)))
    m["err_peak_deg"] = float(np.max(en))
    m["err_rmse_roll_deg"] = float(np.sqrt(np.mean(ex ** 2)))
    m["err_rmse_pitch_deg"] = float(np.sqrt(np.mean(ey ** 2)))
    m["err_rmse_yaw_deg"] = float(np.sqrt(np.mean(ez ** 2)))
    m["err_peak_roll_deg"] = float(np.max(np.abs(ex)))
    m["err_peak_pitch_deg"] = float(np.max(np.abs(ey)))
    m["err_peak_yaw_deg"] = float(np.max(np.abs(ez)))

    # -- 4/5 disturbance window + recovery ----------------------------------
    w = windows.get("dist")
    if w is None:
        for k in ("peak_err_dist_deg", "peak_err_dist_tail_deg",
                  "recovery_time_dist_s", "recovery_time_dist_rel_s",
                  "dist_settle_band_rel_deg"):
            m[k] = None
    else:
        w0, w1 = w
        sel = (t >= w0) & (t <= w1)
        tail = (t >= w0) & (t <= min(w1 + DIST_TAIL_SEC, t[-1]))
        m["peak_err_dist_deg"] = float(np.max(en[sel])) if sel.any() else None
        m["peak_err_dist_tail_deg"] = float(np.max(en[tail])) if tail.any() else None
        pk = m["peak_err_dist_deg"] or 0.0
        m["dist_settle_band_rel_deg"] = RECOVERY_REL_FRAC * pk
        m["recovery_time_dist_s"] = _hold_time(t, en, w1, ERR_TOL_ABS_DEG, HOLD_SEC)
        m["recovery_time_dist_rel_s"] = (_hold_time(t, en, w1,
                                                    RECOVERY_REL_FRAC * pk, HOLD_SEC)
                                         if pk > 0 else None)

    # -- 5/… manoeuvre window ------------------------------------------------
    w = windows.get("man")
    if w is None:
        for k in ("peak_err_man_deg", "maneuver_time_s", "maneuver_completed",
                  "maneuver_tol_deg", "hold_rmse_deg", "hold_peak_deg",
                  "max_overshoot_pitch_deg", "max_overshoot_roll_deg",
                  "max_overshoot_yaw_deg"):
            m[k] = None
    else:
        m0, m1 = w
        step = float(rec.get("maneuver_delta_deg", 0.0) or 0.0)
        sign = 1.0 if step >= 0 else -1.0
        tol = max(MAN_TOL_FLOOR_DEG, MAN_TOL_FRAC * abs(step))
        m["maneuver_tol_deg"] = tol
        sel = (t >= m0) & (t <= m1)
        m["peak_err_man_deg"] = float(np.max(en[sel])) if sel.any() else None
        # Completion means "reoriented AND tracking the new reference": the
        # tracking error must stay inside the band for MAN_HOLD_SEC counted
        # from the END of the reference ramp (m1).  The reference itself keeps
        # moving after the ramp (the pitch reference rides the flight-path
        # angle), so a frozen "final attitude" is not a valid target.
        s = _hold_time(t, en, m1, tol, MAN_HOLD_SEC)
        m["maneuver_settle_after_ref_s"] = s
        m["maneuver_time_s"] = None if s is None else float(s + (m1 - m0))
        m["maneuver_completed"] = s is not None
        h0, h1 = min(m1, t[-1]), min(m1 + HOLD_WINDOW_SEC, t[-1])
        hsel = (t >= h0) & (t <= h1)
        m["hold_rmse_deg"] = float(np.sqrt(np.mean(en[hsel] ** 2))) if hsel.any() else None
        m["hold_peak_deg"] = float(np.max(en[hsel])) if hsel.any() else None
        m["max_overshoot_pitch_deg"] = (float(sign * np.max(sign * ey[hsel]))
                                        if hsel.any() else None)
        m["max_overshoot_roll_deg"] = float(np.max(ex[hsel])) if hsel.any() else None
        m["max_overshoot_yaw_deg"] = float(np.max(ez[hsel])) if hsel.any() else None

    # -- high-altitude (rarefied) window, fixed by the trajectory ------------
    w = windows.get("high")
    if w is None:
        m["high_rmse_deg"] = m["high_peak_deg"] = m["high_time_s"] = None
    else:
        w0, w1 = w
        sel = (t >= w0) & (t <= w1)
        m["high_rmse_deg"] = float(np.sqrt(np.mean(en[sel] ** 2))) if sel.any() else None
        m["high_peak_deg"] = float(np.max(en[sel])) if sel.any() else None
        m["high_time_s"] = float(w1 - w0)
        m["high_window_s"] = [float(w0), float(w1)]

    # -- 6/7/8 aerodynamic surfaces -----------------------------------------
    d = np.asarray(rec["delta_deg"], dtype=float)
    dmax = np.asarray(delta_max_deg, dtype=float) if delta_max_deg is not None else None
    m["delta_peak_deg"] = [float(v) for v in np.max(np.abs(d), axis=0)]
    m["delta_peak_all_deg"] = float(np.max(np.abs(d)))
    if dmax is not None:
        sat = np.abs(d) >= SAT_FRAC * dmax[None, :]
        m["delta_sat_frac"] = [float(v) for v in np.mean(sat, axis=0)]
        m["delta_sat_frac_all"] = float(np.mean(sat.any(axis=1)))
    else:
        m["delta_sat_frac"] = None
        m["delta_sat_frac_all"] = None
    m["delta_travel_deg"] = float(np.sum(np.abs(np.diff(d, axis=0)))) / max(len(d) - 1, 1)
    m["aero_moment_peak_Nm"] = [float(v) for v in np.max(np.abs(rec["M_aero"]), axis=0)]

    # -- 9/10/11/12 CMG ------------------------------------------------------
    Hn = np.asarray(rec["H_norm"], dtype=float)
    Mx = np.asarray(rec["M_cmg"], dtype=float)
    gd = np.asarray(rec["gdot"], dtype=float)
    m["cmg_torque_peak_Nm"] = float(np.max(np.linalg.norm(Mx, axis=1)))
    m["cmg_momentum_peak_Nms"] = float(np.max(Hn))
    m["cmg_momentum_end_Nms"] = float(Hn[-1])
    m["cmg_gimbal_rate_peak_rad_s"] = float(np.max(np.abs(gd)))
    m["cmg_sigma_min"] = float(np.min(rec["sigma"]))
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.0
    m["cmg_impulse_proxy_Nms2"] = float(np.sum(np.abs(Mx)) * dt)

    # -- 13/14/15/16 solver health ------------------------------------------
    fail = np.asarray(rec["qp_fail"], dtype=float) > 0.5
    m["qp_fail_count"] = int(np.sum(fail))
    m["qp_fail_frac"] = float(np.mean(fail))
    m["qp_fail_longest_run"] = _longest_run(fail)
    fb = np.asarray(rec["fallback"], dtype=float) > 0.5
    m["fallback_count"] = int(np.sum(fb))
    m["fallback_longest_run"] = _longest_run(fb)
    m["max_constraint_violation"] = float(np.max(rec["viol"]))
    if "qp_converged" in rec:
        conv = np.asarray(rec["qp_converged"], dtype=float) > 0.5
        it = np.asarray(rec["qp_iter"], dtype=float)
        m["qp_not_converged_count"] = int(np.sum(~conv))
        m["qp_iter_mean"] = float(np.mean(it))
        m["qp_iter_p95"] = float(np.percentile(it, 95))
    else:
        m["qp_not_converged_count"] = None
        m["qp_iter_mean"] = None
        m["qp_iter_p95"] = None
    st = np.asarray(rec["solve_time_ms"], dtype=float)
    m["solve_time_mean_ms"] = float(np.mean(st))
    m["solve_time_median_ms"] = float(np.median(st))
    m["solve_time_p95_ms"] = float(np.percentile(st, 95))
    m["solve_time_max_ms"] = float(np.max(st))
    m["wall_time_s"] = float(rec.get("wall_time_s", float("nan")))

    # -- physical plausibility gates (§10.4) --------------------------------
    m["numeric_ok"] = bool(np.isfinite(m["err_rmse_deg"])
                           and np.isfinite(m["max_constraint_violation"]))
    return m
