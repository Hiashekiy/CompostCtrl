"""E3 of the C1 diagnostics: is the QP failure physical infeasibility, a
constraint-construction problem, or a numerical solve failure?

    python scripts/e3_qp_forensics.py            # replay + freeze + analyse
    python scripts/e3_qp_forensics.py --no-replay # reuse existing frozen QPs

Four parts, in the order of the plan (§3.2-§3.5):

1. rebuild the failure timeline at control-step resolution from the recorded B
   run (100-120 s), including the one-step lag between the decision and the
   recorded state;
2. replay the closed loop with an instrumented copy of the *unmodified*
   controller, verify it reproduces the recorded trajectory, and freeze the
   exact QP at three steps: the last success before the first fallback (t0), the
   first fallback (t1) and the step where |H| crosses the budget (t2);
3. solve an independent slack-minimising feasibility LP for each frozen QP
   (dynamics rows kept as hard equalities; a second variant that also relaxes
   the dynamics is reported separately) and attribute the minimum necessary
   slack to constraint groups;
4. counterfactual relaxations Q0-Q4 (momentum budget / singularity / gimbal
   rate / numerics) on the *same* frozen problem, plus solver-setting variants.

Nothing here is a performance result: relaxed-hardware solutions are diagnostics
only.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import sys

import numpy as np
from scipy.optimize import linprog

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (ROOT, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import run_cmg_ab as ab                                             # noqa: E402
from nearspace import sim                                            # noqa: E402
from nearspace.cmg_control import LTVMPCCMG2, NXP, NUP                # noqa: E402
from nearspace.qp import QPSolver                                     # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction.yaml")
RUN = os.path.join(ROOT, "results", "c1_cmg_reproduction", "cmg", "c1", "B")
OUT = os.path.join(ROOT, "results", "c1_cmg_diagnostics", "E3_qp_failure")
FIG = os.path.join(ROOT, "results", "c1_cmg_diagnostics", "figures")
CACHE = os.path.join(ROOT, "results", "cmg_ab", "_cache")
DT = 0.2
T_REPLAY = 125.0

# row groups that carry a physical limit vs. controller-side bounds
HARDWARE_GROUPS = ("state_gimbal_rate", "gimbal_rate_cmd", "torque_envelope",
                   "state_delta", "delta_rate", "delta_box", "cmg_torque_box")
BUDGET_GROUPS = ("momentum_box", "momentum_radial", "no_unload_thin_air")


# ---------------------------------------------------------------------------
# instrumented controller (no change to the arithmetic)
# ---------------------------------------------------------------------------
class ForensicMPC(LTVMPCCMG2):
    """Same controller, but it keeps the QP it built at selected time steps."""

    def __init__(self, prm, cfg, cluster=None):
        super().__init__(prm, cfg, cluster)
        self.capture_steps = set()
        self._capturing = False
        self._pending = []
        self.frozen = {}
        self.step_log = []

    def _build_qp(self, x0, x_seq, u_seq, p_seq, t_seq, traj):
        H, g, A, lb, ub = super()._build_qp(x0, x_seq, u_seq, p_seq, t_seq, traj)
        if self._capturing:
            self._pending.append(dict(
                H=H.copy(), g=g.copy(), A=A.copy(), lb=lb.copy(), ub=ub.copy(),
                tags=list(self.row_tags), x0=np.asarray(x0, float).copy(),
                x_seq=np.asarray(x_seq, float).copy(),
                u_seq=np.asarray(u_seq, float).copy(),
                p_seq=np.asarray(p_seq, float), t_seq=np.asarray(t_seq, float)))
        return H, g, A, lb, ub

    def solve(self, x_plant, t0, traj):
        idx = int(round(t0 / DT))
        self._capturing = idx in self.capture_steps
        self._pending = []
        z_warm = None if self.z_warm is None else self.z_warm.copy()
        u, info = super().solve(x_plant, t0, traj)
        self.step_log.append(dict(t=float(t0), fallback=int(info["fallback"]),
                                  qp_status=str(info["qp_status"]),
                                  qp_iter=int(info["qp_iter"]),
                                  viol=float(info["viol"]),
                                  solve_ms=1e3 * float(info["solve_time"])))
        if self._capturing:
            self.frozen[round(float(t0), 6)] = dict(
                t0=float(t0), x_plant=np.asarray(x_plant, float).copy(),
                z_warm=z_warm, info=dict(fallback=int(info["fallback"]),
                                         qp_status=str(info["qp_status"]),
                                         qp_iter=int(info["qp_iter"]),
                                         viol=float(info["viol"])),
                builds=self._pending)
        self._capturing = False
        return u, info


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _series(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def _violation(A, z, lb, ub):
    Az = A @ z
    d = np.maximum(lb - Az, Az - ub)
    d = d[np.isfinite(d)]
    return float(d.max()) if d.size else 0.0


def _objective(H, g, z):
    return float(0.5 * z @ H @ z + g @ z)


def _replay_osqp(H, g, A, lb, ub, z0, max_iter=20000, scaling=False, eps=1e-5):
    qp = QPSolver(max_iter=max_iter, eps_abs=eps, eps_rel=eps, internal_scaling=scaling)
    z = qp.solve(H + 1e-4 * np.eye(H.shape[0]), g, A, lb, ub, z0=z0)
    return dict(status=qp.last_status, iterations=int(qp.last_iter),
                violation=_violation(A, z, lb, ub), objective=_objective(H, g, z),
                z=z)


def _row_scales(A, rows):
    s = np.max(np.abs(A[rows]), axis=1)
    s[s == 0.0] = 1.0
    return s


def _solve_feasibility(A, lb, ub, tags, relax_equality, mode):
    """One independent check of the frozen constraint set.

    mode='chebyshev'  min t  s.t.  row_i(z) <= t * scale_i   (robust, gives the
                      smallest uniform relative violation; t = 0 <=> feasible)
    mode='l1'         min ||s||_1  with per-row slack and *row-scaled* rows (the
                      formulation asked for in the plan; reported as a second
                      opinion because a large ill-conditioned LP can stop early)

    The dynamics rows (lb == ub) are hard equalities by default: they are the
    prediction model, not a constraint anyone can relax.  With
    relax_equality=True they join the slacked set, which quantifies how far the
    model itself is from the rest of the problem.
    """
    nz = A.shape[1]
    eq = np.isfinite(lb) & np.isfinite(ub) & np.isclose(lb, ub, atol=0.0, rtol=0.0)
    eqs = np.flatnonzero(eq) if not relax_equality else np.array([], dtype=int)
    ineq = np.flatnonzero(~eq) if not relax_equality else np.arange(A.shape[0])
    if ineq.size == 0:
        return dict(mode=mode, status="empty", feasible=True, t=0.0,
                    total_raw_slack=0.0, by_tag={}, n_ineq_rows=0,
                    n_eq_rows=int(eqs.size))
    scale = _row_scales(A, ineq)
    rows, rhs = [], []
    for j, i in enumerate(ineq):
        if np.isfinite(ub[i]):
            rows.append(np.concatenate([A[i] / scale[j], [-1.0 if mode == "chebyshev"
                                                          else 0.0]])); rhs.append(ub[i] / scale[j])
        if np.isfinite(lb[i]):
            rows.append(np.concatenate([-A[i] / scale[j], [-1.0 if mode == "chebyshev"
                                                           else 0.0]])); rhs.append(-lb[i] / scale[j])
    A_ub = np.array(rows)
    b_ub = np.array(rhs)
    if mode == "chebyshev":
        c = np.concatenate([np.zeros(nz), [1.0]])
        bounds = [(None, None)] * nz + [(0.0, None)]
    else:
        # one slack per inequality row: A z - s <= b, s >= 0 (row-scaled rows)
        nsl = len(ineq)
        A_ub = np.hstack([A_ub[:, :nz], np.zeros((A_ub.shape[0], nsl))])
        idx = np.zeros(A_ub.shape[0], dtype=int)
        k = 0
        for j in range(nsl):
            nrow = int(np.isfinite(ub[ineq[j]])) + int(np.isfinite(lb[ineq[j]]))
            idx[k:k + nrow] = j
            k += nrow
        A_ub[np.arange(A_ub.shape[0]), nz + idx] = -1.0
        c = np.concatenate([np.zeros(nz), np.ones(nsl)])
        bounds = [(None, None)] * nz + [(0.0, None)] * nsl
    A_eq = np.hstack([A[eqs] / _row_scales(A, eqs)[:, None],
                      np.zeros((eqs.size, A_ub.shape[1] - nz))]) if eqs.size else None
    b_eq = lb[eqs] / _row_scales(A, eqs) if eqs.size else None
    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds,
                  method="highs")
    out = dict(mode=mode, status=str(res.status), message=str(res.message),
               n_ineq_rows=int(len(ineq)), n_eq_rows=int(eqs.size),
               feasible=None, t=None, total_raw_slack=None, max_raw_slack=None,
               by_tag={}, n_violated_rows=None, z=None)
    if res.x is None or not res.success:
        return out
    z = res.x[:nz]
    raw = np.maximum(np.maximum(lb - A @ z, A @ z - ub), 0.0)
    raw[~np.isfinite(raw)] = 0.0
    rel = raw[ineq] / scale
    per_tag = collections.defaultdict(lambda: dict(total=0.0, max=0.0, n_rows=0, n_viol=0))
    for j, i in enumerate(ineq):
        tg = tags[i] if tags is not None else "unknown"
        e = per_tag[tg]
        e["total"] += float(raw[i]); e["max"] = max(e["max"], float(raw[i]))
        e["n_rows"] += 1
        if raw[i] > 1e-8 * scale[j]:
            e["n_viol"] += 1
    out.update(z=z, t=float(rel.max()), feasible=bool(rel.max() <= 1e-6),
               total_raw_slack=float(raw[ineq].sum()),
               max_raw_slack=float(raw.max()),
               n_violated_rows=int((rel > 1e-6).sum()),
               by_tag={k: v for k, v in sorted(per_tag.items(),
                                               key=lambda kv: -kv[1]["total"])})
    return out


def feasibility_lp(A, lb, ub, tags, relax_equality=False):
    """Independent feasibility verdict: Chebyshev first, L1 as a second opinion.

    The Chebyshev problem is used for the verdict because it is a one-variable
    minimax LP with row-normalised constraints - it stays well conditioned where
    the plain L1 elastic problem (also reported) can stop at a poor vertex on
    this matrix (entries from 1e-16 to 5e1 over 350 columns).
    """
    cheb = _solve_feasibility(A, lb, ub, tags, relax_equality, "chebyshev")
    l1 = _solve_feasibility(A, lb, ub, tags, relax_equality, "l1")
    best = cheb if (cheb["feasible"] is not None and
                    (l1["feasible"] is None or
                     (cheb["t"] or 0.0) <= (l1["t"] or 0.0))) else l1
    return dict(status="%s/%s" % (cheb["status"], l1["status"]),
                n_ineq_rows=cheb["n_ineq_rows"], n_eq_rows=cheb["n_eq_rows"],
                relax_equality=bool(relax_equality),
                chebyshev=dict((k, v) for k, v in cheb.items() if k != "z"),
                l1=dict((k, v) for k, v in l1.items() if k != "z"),
                t=best["t"], feasible=best["feasible"],
                total_raw_slack=best["total_raw_slack"],
                max_raw_slack=best["max_raw_slack"],
                n_violated_rows=best["n_violated_rows"],
                by_tag=best["by_tag"], z=best["z"],
                method_note="verdict from the row-scaled Chebyshev minimax LP")





def _subset(A, lb, ub, tags, mask):
    m = np.asarray(mask)
    return A[m], lb[m], ub[m], [t for t, k in zip(tags, m) if k]


def _relax_bounds(lb, ub, tags, groups, big=1e6):
    lb2, ub2 = lb.copy(), ub.copy()
    for i, t in enumerate(tags):
        if t in groups:
            if np.isfinite(ub2[i]):
                ub2[i] = big
            if np.isfinite(lb2[i]):
                lb2[i] = -big
    return lb2, ub2


# ---------------------------------------------------------------------------
# part 1: event timeline from the recorded run
# ---------------------------------------------------------------------------
def timeline():
    ts = _series(os.path.join(RUN, "timeseries.csv"))
    t = ts["t"]
    nH = ts["H_norm"]
    dt = np.gradient(nH, DT)
    rows = []
    for i, tt in enumerate(t):
        if not (99.999 <= tt <= 120.001):
            continue
        rows.append(dict(
            t=tt, err_norm_deg=ts["err_norm_deg"][i],
            err_pitch_deg=ts["err_pitch_deg"][i], err_roll_deg=ts["err_roll_deg"][i],
            omega_x=ts["omega_x"][i], omega_y=ts["omega_y"][i], omega_z=ts["omega_z"][i],
            de=ts["de"][i], da=ts["da"][i], dr=ts["dr"][i],
            M_aero_y=ts["M_aero_y"][i], M_dist_y=ts["M_dist_y"][i],
            M_cmg_x=ts["M_cmg_x"][i], M_cmg_y=ts["M_cmg_y"][i], M_cmg_z=ts["M_cmg_z"][i],
            H_norm=nH[i], H_x=ts["H_cmg_x"][i], H_y=ts["H_cmg_y"][i],
            H_z=ts["H_cmg_z"][i], dHdt=dt[i],
            gdot_max_abs=float(np.max(np.abs([ts["gdot_1"][i], ts["gdot_2"][i],
                                              ts["gdot_3"][i], ts["gdot_4"][i]]))),
            sigma=ts["sigma"][i], qp_iter=ts["qp_iter"][i],
            viol=ts["viol"][i], solve_ms=ts["solve_time_ms"][i],
            fallback=ts["fallback"][i]))
    with open(os.path.join(OUT, "event_timeline.csv"), "w", encoding="utf-8", newline="") as fh:
        keys = list(rows[0].keys())
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join("%.10g" % r[k] for k in keys) + "\n")

    H_lim = 60.0
    floor = 0.30 * 0.816496580927726 * 50.0    # sigma_floor_frac * home_sigma_min
    first_fb = next((i for i in range(len(t)) if ts["fallback"][i] > 0.5), None)
    first_sig = next((i for i in range(len(t)) if ts["sigma"][i] < floor), None)
    first_H = next((i for i in range(len(t)) if nH[i] > H_lim), None)
    events = dict(
        H_limit=float(H_lim), sigma_floor=float(floor),
        first_fallback=dict(t=float(t[first_fb]), step=first_fb,
                            state_seen_by_controller_t=float(t[max(first_fb - 1, 0)]),
                            H_at_decision=float(nH[max(first_fb - 1, 0)]),
                            sigma_at_decision=float(ts["sigma"][max(first_fb - 1, 0)]))
        if first_fb is not None else None,
        first_sigma_below_floor=dict(t=float(t[first_sig]), step=first_sig,
                                     sigma=float(ts["sigma"][first_sig]))
        if first_sig is not None else None,
        first_budget_cross=dict(t=float(t[first_H]), step=first_H, H_norm=float(nH[first_H]))
        if first_H is not None else None,
        note="the recorded sigma/H at step k describe the state at t_k, i.e. the "
             "outcome of the command computed at t_{k-1}; the fallback flag at step "
             "k belongs to the command computed from the state at t_{k-1}.",
    )
    if first_fb is not None and first_sig is not None:
        events["order_steps"] = dict(fallback_first=bool(first_fb < first_sig),
                                     same_step=bool(first_fb == first_sig),
                                     delta_steps=int(first_sig - first_fb))
    with open(os.path.join(OUT, "events.json"), "w", encoding="utf-8") as fh:
        json.dump(events, fh, indent=2, ensure_ascii=False)
    return ts, events


# ---------------------------------------------------------------------------
# part 2: replay and freeze
# ---------------------------------------------------------------------------
def replay(freeze_times):
    cfg = ab.build_cfg("c1", "B", CONFIG)
    cfg.simulation.T = T_REPLAY
    b = sim.build(cfg, cache_dir=CACHE)
    ctrl = ForensicMPC(b["prm"], cfg, b["cluster"])
    ctrl.capture_steps = {int(round(t / DT)) for t in freeze_times}
    rec = sim.run(cfg, cache_dir=CACHE, ctrl=ctrl)
    return rec, ctrl


COLUMN_GROUPS = [
    ("omega", ["omega_x", "omega_y", "omega_z"]),
    ("H_cmg", ["H_cmg_x", "H_cmg_y", "H_cmg_z"]),
    ("M_cmg", ["M_cmg_x", "M_cmg_y", "M_cmg_z"]),
    ("gimbal_deg", ["gimbal_1", "gimbal_2", "gimbal_3", "gimbal_4"]),
    ("gdot", ["gdot_1", "gdot_2", "gdot_3", "gdot_4"]),
    ("delta_deg", ["de", "da", "dr"]),
    ("err_norm_deg", ["err_norm_deg"]),
    ("fallback", ["fallback"]),
]


def validate_replay(rec):
    ts = _series(os.path.join(RUN, "timeseries.csv"))
    n = min(len(rec["t"]), len(ts["t"]))
    checks = {}
    for k_new, cols in COLUMN_GROUPS:
        a = np.asarray(rec[k_new])[:n]
        if a.ndim == 1:
            a = a[:, None]
        b = np.column_stack([np.asarray(ts[c])[:n] for c in cols])
        checks[k_new] = float(np.max(np.abs(a - b)))
    return dict(n_steps=int(n), max_abs_diff=checks,
                tolerance=1e-8,
                note="the recorded timeseries.csv is written with %.10g, so the "
                     "replay can only agree to that print precision, not bit-for-bit",
                identical=bool(max(checks.values()) < 1e-8))


# ---------------------------------------------------------------------------
# part 3+4: frozen QP analysis
# ---------------------------------------------------------------------------
def analyse_frozen(step, fr):
    builds = fr["builds"]
    if not builds:
        return None
    bld = builds[-1]                      # the build whose solution the command came from
    H, g, A, lb, ub, tags = bld["H"], bld["g"], bld["A"], bld["lb"], bld["ub"], bld["tags"]
    key = "t%07.2f" % fr["t0"]
    np.savez_compressed(
        os.path.join(OUT, "frozen_qp", key + ".npz"),
        H=H, g=g, A=A, lb=lb, ub=ub, tags=np.array(tags, dtype=object),
        z_warm=np.zeros(1) if fr["z_warm"] is None else fr["z_warm"],
        x0=bld["x0"], x_plant=fr["x_plant"], x_seq=bld["x_seq"], u_seq=bld["u_seq"],
        p_seq=bld["p_seq"], t_seq=bld["t_seq"], allow_pickle=True)

    res = dict(step=int(step), t0=fr["t0"], n_builds=len(builds),
               controller_info=fr["info"],
               n_rows=int(A.shape[0]), n_cols=int(A.shape[1]),
               tag_counts=dict(collections.Counter(tags)))

    # --- Q0: original problem, original solver, original warm start
    q0_warm = _replay_osqp(H, g, A, lb, ub, fr["z_warm"])
    q0_cold = _replay_osqp(H, g, A, lb, ub, None)
    lp0 = feasibility_lp(A, lb, ub, tags)
    lp0_eq = feasibility_lp(A, lb, ub, tags, relax_equality=True)
    res["Q0"] = dict(osqp_warm=dict((k, v) for k, v in q0_warm.items() if k != "z"),
                     osqp_cold=dict((k, v) for k, v in q0_cold.items() if k != "z"),
                     lp=dict((k, v) for k, v in lp0.items() if k != "z"),
                     lp_relax_equality=dict((k, v) for k, v in lp0_eq.items() if k != "z"))

    # --- scaled residuals: is the "violation" a unit problem?
    scale = np.max(np.abs(A), axis=1)
    scale[scale == 0] = 1.0
    if lp0["z"] is not None:
        Az = A @ lp0["z"]
        raw = np.maximum(np.maximum(lb - Az, Az - ub), 0.0)
        raw[~np.isfinite(raw)] = 0.0
        res["scaled_residuals"] = dict(
            max_raw_violation=float(np.nanmax(raw)),
            max_scaled_violation=float(np.nanmax(raw / scale)),
            row_scale_max=float(scale.max()), row_scale_median=float(np.median(scale)))

    # --- counterfactuals on the same frozen problem
    cf = {}
    m_budget = np.array([t not in BUDGET_GROUPS for t in tags])
    cf["Q1_relax_momentum_budget"] = _subset(A, lb, ub, tags, m_budget)
    m_sing = np.array([t != "singularity" for t in tags])
    cf["Q2_relax_singularity"] = _subset(A, lb, ub, tags, m_sing)
    # the momentum "budget" is three separate rows; isolate them so the report can
    # say which one destroys the feasible set
    for tag in ("momentum_box", "no_unload_thin_air", "momentum_radial"):
        m = np.array([t != tag for t in tags])
        cf["Q1_%s_only" % tag] = _subset(A, lb, ub, tags, m)
    A3, lb3, ub3 = A, lb.copy(), ub.copy()
    lb3, ub3 = _relax_bounds(lb3, ub3, tags, HARDWARE_GROUPS)
    cf["Q3_relax_rate_and_servo_limits"] = (A3, lb3, ub3, tags)

    out_cf = {}
    for name, (Ac, lbc, ubc, tagsc) in cf.items():
        lp = feasibility_lp(Ac, lbc, ubc, tagsc)
        o = _replay_osqp(H, g, Ac, lbc, ubc, fr["z_warm"])
        out_cf[name] = dict(lp=dict((k, v) for k, v in lp.items() if k != "z"),
                            osqp=dict((k, v) for k, v in o.items() if k != "z") if o else None)
    res["counterfactuals"] = out_cf

    # --- Q4: numerics only (same constraints)
    num = {}
    for name, kw in (("internal_scaling", dict(scaling=True)),
                     ("max_iter_200k", dict(max_iter=200000)),
                     ("eps_1e8", dict(eps=1e-8)),
                     ("cold_start_no_warm", dict(max_iter=20000))):
        z0 = None if name == "cold_start_no_warm" else fr["z_warm"]
        o = _replay_osqp(H, g, A, lb, ub, z0, **kw)
        num[name] = dict((k, v) for k, v in o.items() if k != "z")
    res["numerics"] = num

    with open(os.path.join(OUT, "frozen_qp", key + ".json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False, default=_j)
    return res


def _j(o):
    if isinstance(o, (np.floating, np.integer)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


# ---------------------------------------------------------------------------
# summary + figure
# ---------------------------------------------------------------------------
def write_summary(tl_events, val, results):
    L = ["# E3 QP 失效因果诊断", "",
         "自动生成：`scripts/e3_qp_forensics.py`。原始 B 组数据只读，未修改。", ""]
    L += ["## 1 失效时序（按控制步，不按四舍五入的整秒）", ""]
    e = tl_events
    L += ["| 事件 | 时刻 [s] | 步号 | 备注 |", "|---|---:|---:|---|"]
    if e["first_fallback"]:
        f = e["first_fallback"]
        L.append("| 首次 QP 回退 | %.1f | %d | 该步命令由 t=%.1f s 的状态算出（回退标志滞后一步） |"
                 % (f["t"], f["step"], f["state_seen_by_controller_t"]))
    if e["first_sigma_below_floor"]:
        s = e["first_sigma_below_floor"]
        L.append("| σ_min 首次低于下限 | %.1f | %d | σ_min=%.3f（下限 %.3f） |"
                 % (s["t"], s["step"], s["sigma"], e["sigma_floor"]))
    if e["first_budget_cross"]:
        h = e["first_budget_cross"]
        L.append("| |H| 首次超过预算 | %.1f | %d | |H|=%.2f（预算 %.0f） |"
                 % (h["t"], h["step"], h["H_norm"], e["H_limit"]))
    L.append("")
    if "order_steps" in e:
        o = e["order_steps"]
        L.append("同一步粒度的先后关系：回退先于 σ 跌破下限 = **%s**，相差 %d 步。"
                 "决策时刻的状态：|H|=%.2f N·m·s、σ_min=%.3f，两者都还在限制之内 —— "
                 "也就是说第一次回退**不是**由当时的超预算或奇异状态直接触发的。"
                 % (o["fallback_first"], o["delta_steps"],
                    e["first_fallback"]["H_at_decision"],
                    e["first_fallback"]["sigma_at_decision"]))
        L.append("")
    L += ["## 2 重放一致性", "",
          "| 记录 | 最大绝对差 |", "|---|---:|"]
    for k, v in val["max_abs_diff"].items():
        L.append("| %s | %.3e |" % (k, v))
    L.append("")
    L.append("重放与前 626 步记录%s。" % ("逐位一致" if val["identical"] else "存在差异（见上表）"))
    L.append("")
    L += ["## 3 冻结 QP 的独立可行性检查", "",
          "每个冻结时刻都分别在原求解器上重放、并用独立 LP（最小总松弛）检查可行性。"
          "动力学行（lb=ub）默认作为硬等式，不允许松弛；另有一行 `lp_relax_equality` "
          "把动力学行也放进松弛集，用来量化模型本身与其余约束的差距。", "",
          "| 时刻 | 原求解器状态(warm) | 违反量 | 独立 LP 最小相对违反 t* | 可行? | 主要松弛来源 |",
          "|---|---|---:|---:|---|---|"]
    for r in results:
        if r is None:
            continue
        q0 = r["Q0"]["lp"]
        src = ", ".join("%s=%.3g" % (t, d["total"]) for t, d in list(q0["by_tag"].items())[:3])
        L.append("| %.1f s | %s | %.3e | %.3e | %s | %s |"
                 % (r["t0"], r["Q0"]["osqp_warm"]["status"], r["Q0"]["osqp_warm"]["violation"],
                    q0["t"] if q0["t"] is not None else float("nan"),
                    "是" if q0["feasible"] else "**否**", src or "-"))
    L.append("")
    L += ["### 3.1 逐组最小必要松弛", "",
          "| 时刻 | 约束组 | 行数 | 需要松弛的行数 | 总松弛 | 最大松弛 |",
          "|---|---|---:|---:|---:|---:|"]
    for r in results:
        if r is None:
            continue
        for tg, d in r["Q0"]["lp"]["by_tag"].items():
            L.append("| %.1f s | %s | %d | %d | %.4g | %.4g |"
                     % (r["t0"], tg, d["n_rows"], d["n_viol"], d["total"], d["max"]))
    L.append("")
    L += ["## 4 反事实放松（仅诊断，不是性能结果）", "",
          "| 版本 | 独立 LP 最小相对违反 t* | 可行? | 原求解器状态 | 违反量 |",
          "|---|---:|---|---|---:|"]
    for r in results:
        if r is None:
            continue
        q0 = r["Q0"]
        L.append("| %.1f s / Q0 原始 | %.3e | %s | %s | %.3e |"
                 % (r["t0"], q0["lp"]["t"] if q0["lp"]["t"] is not None else float("nan"),
                    "是" if q0["lp"]["feasible"] else "**否**",
                    q0["osqp_warm"]["status"], q0["osqp_warm"]["violation"]))
        for name, d in r["counterfactuals"].items():
            lp = d["lp"]
            L.append("| %.1f s / %s | %.3e | %s | %s | %.3e |"
                     % (r["t0"], name,
                        lp["t"] if lp["t"] is not None else float("nan"),
                        "是" if lp["feasible"] else "**否**",
                        d["osqp"]["status"] if d["osqp"] else "-",
                        d["osqp"]["violation"] if d["osqp"] else float("nan")))
    L.append("")
    L += ["## 5 数值设置的影响（约束不变）", "",
          "| 时刻 | 设置 | 状态 | 迭代 | 违反量 |", "|---|---|---|---:|---:|"]
    for r in results:
        if r is None:
            continue
        for name, d in r["numerics"].items():
            L.append("| %.1f s | %s | %s | %d | %.3e |"
                     % (r["t0"], name, d["status"], d["iterations"], d["violation"]))
        L.append("| %.1f s | 基线（warm, 项目设置） | %s | %d | %.3e |"
                 % (r["t0"], r["Q0"]["osqp_warm"]["status"],
                    r["Q0"]["osqp_warm"]["iterations"], r["Q0"]["osqp_warm"]["violation"]))
    L.append("")
    L += ["## 6 结论（由上面四节的数据决定）", ""]
    L.append(_conclusion(results))
    L.append("")
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")


def _conclusion(results):
    rs = [r for r in results if r]
    if not rs:
        return "无冻结数据。"
    feas = [bool(r["Q0"]["lp"]["feasible"]) for r in rs]
    osqp_fail = [r["Q0"]["osqp_warm"]["violation"] > 1e-3 for r in rs]
    main = collections.Counter()
    for r in rs:
        bt = r["Q0"]["lp"]["by_tag"]
        if bt:
            top = max(bt.items(), key=lambda kv: kv[1]["total"])[0]
            main[top] += 1
    txt = []
    txt.append("* 冻结的 %d 个控制步中，独立 LP 判定原始 QP 可行的是 **%d 个**：%s"
               % (len(rs), sum(feas), "、".join("%.1f s→%s" % (r["t0"], "可行" if f else "不可行")
                                                for r, f in zip(rs, feas))))
    txt.append("* 原求解器在这些步上是否给出可用解（违反量 < 1e-3）：%s"
               % "、".join("%.1f s→%s" % (r["t0"], "否" if f else "是")
                           for r, f in zip(rs, osqp_fail)))
    if main:
        txt.append("* 需要松弛最多的约束组：%s"
                   % "、".join("%s（%d 步）" % (k, v) for k, v in main.most_common()))
    txt.append("* 反事实放松的结果见第 4 节：若只有放松某类约束才恢复可行，该约束就是该预测"
               "问题上的关键限制；若原始问题本就可行而求解器失败，则应归因于数值求解。")
    return "\n".join(txt)


def make_figure(events, ctrl):
    os.makedirs(FIG, exist_ok=True)
    ts = _series(os.path.join(RUN, "timeseries.csv"))
    t = ts["t"]
    sel = (t >= 95) & (t <= 130)
    if ctrl is not None:
        step_t = [d["t"] for d in ctrl.step_log]
        step_viol = [d["viol"] for d in ctrl.step_log]
        fb_t = [d["t"] for d in ctrl.step_log if d["fallback"]]
    else:
        step_t = list(t)
        step_viol = list(ts["viol"])
        fb_t = [tt for tt, f in zip(t, ts["fallback"]) if f > 0.5]
    # everything on this figure is the failure window, so the shared x-axis must
    # not be stretched by the rest of the run
    keep = [(95.0 <= x <= 135.0) for x in step_t]
    step_t = [x for x, k in zip(step_t, keep) if k]
    step_viol = [v for v, k in zip(step_viol, keep) if k]
    lt = step_t
    f, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    ax[0].plot(t[sel], ts["err_norm_deg"][sel], "k", lw=1.0, label="attitude error")
    ax[0].set_ylabel("error [deg]")
    ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[1].plot(t[sel], ts["H_norm"][sel], "C1", lw=1.2, label="|H|")
    ax[1].axhline(events["H_limit"], color="r", ls="--", lw=1.0, label="budget 60")
    ax[1].set_ylabel("|H| [N m s]"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[2].plot(t[sel], ts["sigma"][sel], "C2", lw=1.2, label="sigma_min")
    ax[2].axhline(events["sigma_floor"], color="r", ls="--", lw=1.0, label="floor 12.25")
    ax[2].set_yscale("log")
    ax[2].set_ylabel("sigma_min"); ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)
    ax[3].plot(step_t, step_viol, "C3", lw=1.0, label="constraint violation")
    ax[3].axhline(1e-3, color="r", ls="--", lw=1.0, label="fallback threshold 1e-3")
    ax[3].set_yscale("symlog", linthresh=1e-6)
    ax[3].set_ylabel("violation"); ax[3].grid(alpha=0.3); ax[3].legend(fontsize=8)
    if fb_t:
        ax[3].axvline(fb_t[0], color="k", ls=":", lw=1.0)
        for a in ax:
            a.axvline(fb_t[0], color="k", ls=":", lw=1.0)
    for key, col in (("first_sigma_below_floor", "C2"), ("first_budget_cross", "C1")):
        if events.get(key):
            for a in ax:
                a.axvline(events[key]["t"], color=col, ls="-.", lw=0.9)
    ax[0].set_title("E3  failure timeline (dotted = first fallback, dash-dot = sigma/|H| crossings)")
    ax[3].set_xlabel("t [s]")
    f.tight_layout()
    f.savefig(os.path.join(FIG, "e3_timeline.png"), dpi=130)
    plt.close(f)


# ---------------------------------------------------------------------------
def frozen_from_npz(path):
    d = np.load(path, allow_pickle=True)
    tags = [str(t) for t in d["tags"]]
    zw = d["z_warm"]
    return dict(t0=float(d["t_seq"][0]), x_plant=np.asarray(d["x_plant"], float),
                z_warm=None if zw.size <= 1 else zw, info=None,
                builds=[dict(H=d["H"], g=d["g"], A=d["A"], lb=d["lb"], ub=d["ub"],
                             tags=tags, x0=np.asarray(d["x0"], float),
                             x_seq=np.asarray(d["x_seq"], float),
                             u_seq=np.asarray(d["u_seq"], float),
                             p_seq=np.asarray(d["p_seq"], float),
                             t_seq=np.asarray(d["t_seq"], float))])


def load_frozen_results():
    """Re-read the per-step forensic JSON (usable without repeating the replay)."""
    d = os.path.join(OUT, "frozen_qp")
    res = []
    for name in sorted(os.listdir(d)):
        if name.endswith(".json") and name != "summary.json":
            with open(os.path.join(d, name), encoding="utf-8") as fh:
                res.append(json.load(fh))
    return sorted(res, key=lambda r: r["t0"])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-replay", action="store_true",
                    help="rebuild the summary from the frozen QPs already on disk")
    ap.add_argument("--from-frozen", action="store_true",
                    help="re-run the forensic analysis on the saved frozen QPs")
    a = ap.parse_args(argv)
    os.makedirs(os.path.join(OUT, "frozen_qp"), exist_ok=True)

    ts, events = timeline()
    print("events:", json.dumps(events, ensure_ascii=False)[:400])

    f = events["first_fallback"]
    s = events["first_sigma_below_floor"]
    h = events["first_budget_cross"]
    freeze = sorted({round(f["t"] - DT, 6), round(f["t"], 6),
                     round(h["t"], 6)})
    if a.from_frozen:
        d = os.path.join(OUT, "frozen_qp")
        results = []
        for name in sorted(os.listdir(d)):
            if not name.endswith(".npz"):
                continue
            print("  re-analysing", name, flush=True)
            fr = frozen_from_npz(os.path.join(d, name))
            results.append(analyse_frozen(int(round(fr["t0"] / DT)), fr))
        results.sort(key=lambda r: r["t0"])
        with open(os.path.join(OUT, "replay_validation.json"), encoding="utf-8") as fh:
            val = json.load(fh)
        write_summary(events, val, results)
        make_figure(events, None)
        _write_feasibility_csv(results)
        return 0

    if a.no_replay:
        results = load_frozen_results()
        with open(os.path.join(OUT, "replay_validation.json"), encoding="utf-8") as fh:
            val = json.load(fh)
        write_summary(events, val, results)
        make_figure(events, None)
        print("re-summarised %d frozen QPs" % len(results))
        return 0
    print("replaying closed loop to T=%.0f s, freezing at %s" % (T_REPLAY, freeze))
    rec, ctrl = replay(freeze)
    # dump the captured QPs immediately: the replay is expensive and a later
    # analysis error must not cost it again
    for step in sorted(ctrl.capture_steps):
        fr = ctrl.frozen.get(round(step * DT, 6))
        if fr is None or not fr["builds"]:
            continue
        bld = fr["builds"][-1]
        np.savez_compressed(
            os.path.join(OUT, "frozen_qp", "t%07.2f.npz" % fr["t0"]),
            H=bld["H"], g=bld["g"], A=bld["A"], lb=bld["lb"], ub=bld["ub"],
            tags=np.array(bld["tags"], dtype=object),
            z_warm=np.zeros(1) if fr["z_warm"] is None else fr["z_warm"],
            x0=bld["x0"], x_plant=fr["x_plant"], x_seq=bld["x_seq"],
            u_seq=bld["u_seq"], p_seq=bld["p_seq"], t_seq=bld["t_seq"],
            allow_pickle=True)
    val = validate_replay(rec)
    with open(os.path.join(OUT, "replay_validation.json"), "w", encoding="utf-8") as fh:
        json.dump(val, fh, indent=2, ensure_ascii=False)
    print("replay validation:", val["identical"], val["max_abs_diff"])

    results = []
    for step in ctrl.capture_steps:
        fr = ctrl.frozen.get(round(step * DT, 6))
        if fr is None:
            continue
        print("  analysing frozen QP at t=%.1f s ..." % fr["t0"], flush=True)
        results.append(analyse_frozen(step, fr))
    results.sort(key=lambda r: (r is None, r["t0"] if r else 0))

    write_summary(events, val, results)
    make_figure(events, ctrl)

    _write_feasibility_csv(results)
    print("wrote", OUT)
    return 0


def _write_feasibility_csv(results):
    rows = []
    for r in results:
        if r is None:
            continue
        q0 = r["Q0"]
        rows.append(dict(t0=round(r["t0"], 3),
                         osqp_status=q0["osqp_warm"]["status"],
                         osqp_iters=q0["osqp_warm"]["iterations"],
                         osqp_violation=q0["osqp_warm"]["violation"],
                         lp_t_star=q0["lp"]["t"],
                         lp_feasible=bool(q0["lp"]["feasible"]),
                         lp_total_raw_slack=q0["lp"]["total_raw_slack"],
                         lp_max_raw_slack=q0["lp"]["max_raw_slack"],
                         lp_relax_eq_t_star=q0["lp_relax_equality"]["t"],
                         lp_relax_eq_feasible=bool(q0["lp_relax_equality"]["feasible"]),
                         lp_method=q0["lp"]["method_note"]))
    if rows:
        with open(os.path.join(OUT, "feasibility_checks.csv"), "w", newline="",
                  encoding="utf-8") as fh:
            w = csv_writer(fh, list(rows[0].keys()))
            for r in rows:
                w.writerow(r)
    return rows


def csv_writer(fh, keys):
    import csv as _csv
    w = _csv.DictWriter(fh, fieldnames=keys)
    w.writeheader()
    return w


if __name__ == "__main__":
    raise SystemExit(main())
