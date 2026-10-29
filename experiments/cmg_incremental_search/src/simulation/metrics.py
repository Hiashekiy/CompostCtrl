"""指标（指导实施文件 §10）。所有指标都从时序日志的原始数组计算，不接受手工数据。"""

from __future__ import annotations

import numpy as np


def _rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x ** 2))) if x.size else float("nan")


def settle_time(t, err_deg, tol_deg: float) -> float | None:
    """最后一次超过容差的时刻（之后一直保持），无则 0。"""
    t = np.asarray(t, dtype=float)
    bad = np.flatnonzero(np.asarray(err_deg, dtype=float) > tol_deg)
    if bad.size == 0:
        return 0.0
    if bad[-1] >= len(t) - 1:
        return None
    return float(t[bad[-1] + 1])


def compute_metrics(arr: dict, tol_deg: float = 0.5) -> dict:
    if not arr or "t" not in arr:
        return {}
    t = np.asarray(arr["t"], dtype=float)
    err = np.asarray(arr["err_norm_deg"], dtype=float)
    m: dict[str, float | int | None | bool] = {}
    m["n_steps"] = int(t.size)
    m["duration_s"] = float(t[-1] - t[0] + (t[1] - t[0] if t.size > 1 else 0.0))

    m["att_rmse_deg"] = _rms(err)
    m["att_peak_deg"] = float(np.max(err))
    m["att_final_deg"] = float(err[-1])
    m["t_settle_s"] = settle_time(t, err, tol_deg)
    m["success"] = bool(err[-1] <= tol_deg and np.max(err) < 10.0 * max(tol_deg, 1e-9))

    m["tau_req_rms_nm"] = _rms(arr.get("tau_req"))
    m["residual_rms_nm"] = _rms(arr.get("residual_norm", [np.nan]))
    m["residual_max_nm"] = float(np.nanmax(arr.get("residual_norm", [np.nan])))
    gap = np.asarray(arr.get("tau_pred", np.zeros((1, 3))), dtype=float) \
        - np.asarray(arr.get("tau_actual", np.zeros((1, 3))), dtype=float)
    m["torque_gap_mean_nm"] = float(np.nanmean(np.linalg.norm(np.atleast_2d(gap), axis=1))) \
        if gap.size else float("nan")
    m["torque_actual_rms_nm"] = _rms(np.linalg.norm(
        np.atleast_2d(np.asarray(arr.get("tau_actual", np.zeros((1, 3))), dtype=float)), axis=1))
    if "tau_effective" in arr:
        eff = np.atleast_2d(np.asarray(arr["tau_effective"], dtype=float))
        gap_eff = np.atleast_2d(np.asarray(arr.get("tau_pred", np.zeros_like(eff)), dtype=float)) - eff
        m["torque_eff_gap_mean_nm"] = float(np.nanmean(np.linalg.norm(gap_eff, axis=1)))
        m["tau_effective_rms_nm"] = _rms(np.linalg.norm(eff, axis=1))

    m["aero_sat_frac"] = float(np.nanmean(arr.get("aero_sat_frac", [0.0])))
    m["gdot_peak_rad_s"] = float(np.nanmax(np.abs(np.atleast_2d(arr.get("gdot", [[0.0]])))))
    gd = np.atleast_2d(np.asarray(arr.get("gdot", np.zeros((1, 4))), dtype=float))
    if gd.shape[0] > 1:
        m["gddot_peak_rad_s2"] = float(np.max(np.abs(np.diff(gd, axis=0) / np.mean(np.diff(t)))))
    else:
        m["gddot_peak_rad_s2"] = 0.0
    gam = np.atleast_2d(np.asarray(arr.get("gamma", np.zeros((1, 4))), dtype=float))
    m["gamma_max_deg"] = float(np.rad2deg(np.max(np.abs(gam))))
    m["sigma_min_min"] = float(np.nanmin(arr.get("sigma_min", [np.nan])))
    m["H_norm_max"] = float(np.nanmax(arr.get("H_norm", [np.nan])))

    for k, out in (("solve_time_ms", "alloc_ms"), ("ctrl_time_ms", "ctrl_ms"),
                   ("search_time_ms", "search_ms")):
        v = np.asarray(arr.get(k, [np.nan]), dtype=float)
        m[f"{out}_mean"] = float(np.nanmean(v))
        m[f"{out}_max"] = float(np.nanmax(v))

    m["infeasible_count"] = int(np.sum(~np.asarray(
        arr.get("request_feasible", arr.get("feasible", [True])), dtype=bool)))
    m["fallback_count"] = int(np.sum(np.asarray(arr.get("fallback", [False]), dtype=bool)))
    m["timeout_count"] = int(np.sum(np.asarray(arr.get("search_status", [""])) == "timeout"))
    return m
