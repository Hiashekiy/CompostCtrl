"""Replay the C0-C4 / T1-T3 experiment groups with the momentum device as an axis.

    python scripts/run_device_sweep.py --all --jobs 4
    python scripts/run_device_sweep.py --case cseries --arms C1 C3_rw C3_cmg
    python scripts/run_device_sweep.py --list

Two historical tasks (configs/cmg_ab/device_sweep.yaml):

    cseries   100 km 顶点，俯仰 5° 余弦脉冲机动 @105 s / 10 s，200 s   (C0-C4)
    tseries   100 km 顶点，5° 俯仰 + 5° 滚转 + 3° 偏航脉冲 @110 s / 20 s (T1/T3)

Each task is replayed for several **arms** (nearspace/arms.py): no device,
reaction wheel, CMG, and the C-group's control-structure variants
(idealised lag, hierarchical allocation, 20 % aero mismatch).

Output: results/cmg_ab/device_sweep/<case>/<arm>/
    timeseries.csv  metrics.json  meta.json  config_resolved.yaml
plus results/cmg_ab/device_sweep/<case>/metrics.csv and summary.md.
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import argparse
import csv
import json
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from nearspace import arms, sim                                    # noqa: E402
from nearspace.config import _to_ns, deep_merge, dump, read_yaml    # noqa: E402
from nearspace.metrics import SCORING, compute                      # noqa: E402
from nearspace.model6 import quat_geodesic, quat_to_euler, wrap_pi   # noqa: E402
from nearspace.model6c import aero_moment6c                         # noqa: E402
from nearspace.scenarios import cmg_spec, controller_budgets, effective_weights
from nearspace.trajectory3 import Trajectory3D                      # noqa: E402
from nearspace.atmosphere_factory import make_atmosphere            # noqa: E402

SWEEP = os.path.join(ROOT, "configs", "cmg_ab", "device_sweep.yaml")
OUTROOT = os.path.join(ROOT, "results", "cmg_ab", "device_sweep")

COLUMNS = [
    "t", "h", "V", "qinf",
    "phi_ref", "theta_ref", "psi_ref", "phi", "theta", "psi",
    "err_roll_deg", "err_pitch_deg", "err_yaw_deg", "err_norm_deg",
    "omega_x", "omega_y", "omega_z",
    "de_cmd", "da_cmd", "dr_cmd", "de", "da", "dr",
    "M_aero_x", "M_aero_y", "M_aero_z",
    "dev_M_x", "dev_M_y", "dev_M_z",
    "dev_H_x", "dev_H_y", "dev_H_z", "dev_H_norm",
    "dev_rate", "sigma",
    "qp_fail", "fallback", "qp_iter", "qp_converged", "viol", "solve_time_ms",
    "M_dist_x", "M_dist_y", "M_dist_z", "maneuver_active",
]


# ---------------------------------------------------------------------------
def load_sweep(path=SWEEP):
    raw = read_yaml(path)
    blocks = {k: raw.pop(k, {}) for k in ("common", "cases", "arms")}
    return raw, blocks


def resolve_case(blocks, case):
    """Expand `extends:` chains inside the cases block (same semantics as the
    A/B matrix, so a sweep case inherits the reference and only overrides what
    it declares)."""
    cases = blocks.get("cases", {})
    if case not in cases:
        raise SystemExit("unknown case %r; known: %s" % (case, ", ".join(sorted(cases))))
    chain, cur = [], case
    while True:
        chain.append(cur)
        parent = cases[cur].get("extends")
        if not parent:
            break
        if parent not in cases or parent in chain:
            raise SystemExit("bad extends chain for %r" % case)
        cur = parent
    merged = {}
    for name in reversed(chain):
        merged = deep_merge(merged, cases[name])
    merged.pop("extends", None)
    return merged


def build_cfg(case, path=SWEEP):
    base, blocks = load_sweep(path)
    d = deep_merge(base, blocks["common"])
    d = deep_merge(d, resolve_case(blocks, case))
    cfg = _to_ns(d)
    sc = cfg.scenario
    cfg.trajectory.amp = float(sc.vertex_km) * 1000.0 - float(cfg.trajectory.h0)
    cfg.simulation.T = float(sc["T"]) if isinstance(sc, dict) else float(sc.T)
    cfg.scenario.name = case
    return cfg


def out_dir(case, arm):
    return os.path.join(OUTROOT, case, arm)


# ---------------------------------------------------------------------------
def run_one(case, arm, verbose=True):
    t0 = time.time()
    cfg = build_cfg(case)
    b = arms.build(cfg, arm)
    atm = make_atmosphere(cfg, cache_dir=os.path.join(ROOT, "results", "_cache"),
                          backend=getattr(cfg.environment, "backend", "msis2"))
    traj = Trajectory3D(cfg, atm)
    prm = b["prm_plant"]
    x = arms.initial_state(b, traj)
    x0 = x.copy()

    T = float(cfg.simulation.T)
    dtc = float(cfg.simulation.dt_ctrl)
    dtp = float(cfg.simulation.dt_plant)
    nsub = max(1, int(round(dtc / dtp)))
    dmax_deg = np.rad2deg(prm.veh.delta_max) if b["kind"] == "cmg" \
        else np.rad2deg(prm.delta_max)

    if verbose:
        print("=" * 78)
        print("case=%s  arm=%s  %s" % (case, arm, b["label"]))
        print("  weights in force :", json.dumps(effective_weights(cfg), ensure_ascii=False))
        if b["kind"] == "cmg":
            print("  controller budget:", json.dumps(controller_budgets(cfg, prm), ensure_ascii=False))
            print("  CMG hardware     :", json.dumps(cmg_spec(cfg, prm), ensure_ascii=False))
        else:
            print("  wheel: h_max=%.1f N·m·s  M_stall=%.1f N·m  P_max=%.0f W  "
                  "tau_cmd_max=%.1f N·m"
                  % (prm.h_max, prm.M_stall, prm.P_max, prm.tau_cmd_max))
        print("  horizon          : T=%.0f s  dt_ctrl=%.3f  dt_plant=%.4f"
              % (T, dtc, dtp))
        print("=" * 78, flush=True)

    rows = {k: [] for k in COLUMNS}
    wall0 = time.perf_counter()
    t = 0.0
    nfail = 0

    def record(tau, u, info):
        eul = quat_to_euler(x[0:4])
        eref = traj.euler_ref(tau)
        d = np.array([wrap_pi(eul[i] - eref[i]) for i in range(3)])
        p = traj.p_array(tau, prm.veh if b["kind"] == "cmg" else prm)
        Ma = (aero_moment6c(x, p, prm) if b["kind"] == "cmg"
              else arms.aero_moment(x, p, prm))
        v = b["view"](x, u)
        failed = int(info.get("fallback", 0))
        rows["t"].append(tau)
        rows["h"].append(traj.h(tau) / 1000.0)
        rows["V"].append(traj.V(tau))
        rows["qinf"].append(traj.qinf(tau))
        rows["phi_ref"].append(np.rad2deg(eref[0]))
        rows["theta_ref"].append(np.rad2deg(eref[1]))
        rows["psi_ref"].append(np.rad2deg(eref[2]))
        rows["phi"].append(np.rad2deg(eul[0]))
        rows["theta"].append(np.rad2deg(eul[1]))
        rows["psi"].append(np.rad2deg(eul[2]))
        rows["err_roll_deg"].append(np.rad2deg(d[0]))
        rows["err_pitch_deg"].append(np.rad2deg(d[1]))
        rows["err_yaw_deg"].append(np.rad2deg(d[2]))
        rows["err_norm_deg"].append(np.rad2deg(quat_geodesic(traj.q_ref(tau), x[0:4])))
        for i, nm in enumerate(("omega_x", "omega_y", "omega_z")):
            rows[nm].append(x[4 + i])
        for i, nm in enumerate(("de_cmd", "da_cmd", "dr_cmd")):
            rows[nm].append(np.rad2deg(u[i]))
        for i, nm in enumerate(("de", "da", "dr")):
            rows[nm].append(np.rad2deg(x[7 + i]))
        for i, nm in enumerate(("M_aero_x", "M_aero_y", "M_aero_z")):
            rows[nm].append(Ma[i])
        for i, nm in enumerate(("dev_M_x", "dev_M_y", "dev_M_z")):
            rows[nm].append(v["dev_M"][i])
        for i, nm in enumerate(("dev_H_x", "dev_H_y", "dev_H_z")):
            rows[nm].append(v["dev_H"][i])
        rows["dev_H_norm"].append(float(np.linalg.norm(v["dev_H"])))
        rows["dev_rate"].append(v["dev_rate"])
        rows["sigma"].append(v["sigma"])
        rows["qp_fail"].append(failed)
        rows["fallback"].append(failed)
        rows["qp_iter"].append(float(info.get("qp_iter", 0)))
        rows["qp_converged"].append(
            float(str(info.get("qp_status", "")).lower().startswith("solved")))
        rows["viol"].append(float(info.get("viol", 0.0)))
        rows["solve_time_ms"].append(1e3 * float(info.get("solve_time", 0.0)))
        for i, nm in enumerate(("M_dist_x", "M_dist_y", "M_dist_z")):
            rows[nm].append(p[4 + i])
        ph = [mv.phase(tau) for mv in traj.maneuvers]
        rows["maneuver_active"].append(int(1 in ph))

    record(0.0, np.zeros(b["nu"]), {})
    # unified MPC exposes .solve(), the hierarchical controller exposes .act()
    call = getattr(b["ctrl"], "solve", None) or b["ctrl"].act
    while t < T - 1e-9:
        u, info = call(x, t, traj)
        for _ in range(nsub):
            p = traj.p_array(t, prm.veh if b["kind"] == "cmg" else prm)
            x = b["step"](x, u, p, dtp)
            t += dtp
        record(t, u, info)
        nfail += int(info.get("fallback", 0))
        if verbose and len(rows["t"]) % 100 == 0:
            print("  t=%6.1f s  err=%.3f deg  |H|=%.1f  fail=%d"
                  % (t, rows["err_norm_deg"][-1], rows["dev_H_norm"][-1], nfail),
                  flush=True)

    wall = time.perf_counter() - wall0
    rec = {k: np.array(v, dtype=float) for k, v in rows.items()}
    rec["q"] = None
    # ---- metrics: reuse the A/B metric definitions -----------------------
    mrec = dict(t=rec["t"], err_norm_deg=rec["err_norm_deg"],
                err_roll_deg=rec["err_roll_deg"], err_pitch_deg=rec["err_pitch_deg"],
                err_yaw_deg=rec["err_yaw_deg"],
                phi=rec["phi"], theta=rec["theta"], psi=rec["psi"],
                phi_ref=rec["phi_ref"], theta_ref=rec["theta_ref"],
                psi_ref=rec["psi_ref"],
                delta_deg=np.column_stack([rec["de"], rec["da"], rec["dr"]]),
                M_aero=np.column_stack([rec["M_aero_x"], rec["M_aero_y"],
                                        rec["M_aero_z"]]),
                M_cmg=np.column_stack([rec["dev_M_x"], rec["dev_M_y"],
                                       rec["dev_M_z"]]),
                H_norm=rec["dev_H_norm"],
                gdot=rec["dev_rate"].reshape(-1, 1),
                sigma=np.nan_to_num(rec["sigma"], nan=1.0e3),
                qp_fail=rec["qp_fail"], fallback=rec["fallback"],
                qp_iter=rec["qp_iter"], qp_converged=rec["qp_converged"],
                viol=rec["viol"], solve_time_ms=rec["solve_time_ms"],
                maneuver_delta_deg=np.rad2deg(traj.maneuver.delta))
    windows = traj.windows(T)
    metrics = compute(mrec, windows, dmax_deg)
    metrics["wall_time_s"] = wall
    metrics["arm"] = arm
    metrics["case"] = case
    metrics["device"] = b["kind"]
    metrics["label"] = b["label"]

    # ---- write -----------------------------------------------------------
    od = out_dir(case, arm)
    os.makedirs(od, exist_ok=True)
    with open(os.path.join(od, "timeseries.csv"), "w", newline="", encoding="utf-8") as fh:
        fh.write(",".join(COLUMNS) + "\n")
        arr = np.column_stack([rec[c] for c in COLUMNS])
        for row in arr:
            fh.write(",".join("%.10g" % v for v in row) + "\n")
    with open(os.path.join(od, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, ensure_ascii=False, default=float)
    meta = dict(case=case, arm=arm, device=b["kind"], label=b["label"],
                code_commit=sim._git_commit(), python=sys.version.split()[0],
                finished_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                wall_time_s=wall, scoring=SCORING,
                windows=dict((k, list(v)) for k, v in windows.items()),
                initial_state=[float(v) for v in x0],
                final_state=[float(v) for v in x],
                n_states=int(len(x0)), config=dump(cfg))
    with open(os.path.join(od, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False, default=float)
    with open(os.path.join(od, "config_resolved.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(dump(cfg), fh, allow_unicode=True, sort_keys=False)

    if verbose:
        print("[%s/%s] RMSE=%.4f deg  peak=%.3f deg  device|M|max=%.2f N·m  "
              "|H|max=%.1f N·m·s  sat=%.3f  QP fail=%d  wall=%.0fs"
              % (case, arm, metrics["err_rmse_deg"], metrics["err_peak_deg"],
                 metrics["cmg_torque_peak_Nm"], metrics["cmg_momentum_peak_Nms"],
                 metrics["delta_sat_frac_all"] or 0.0, metrics["qp_fail_count"], wall),
              flush=True)
    return metrics


# ---------------------------------------------------------------------------
KEY = ["err_rmse_deg", "err_peak_deg", "err_rmse_roll_deg", "err_rmse_pitch_deg",
       "err_rmse_yaw_deg", "peak_err_man_deg", "maneuver_time_s",
       "delta_peak_all_deg", "delta_sat_frac_all", "delta_travel_deg",
       "cmg_torque_peak_Nm", "cmg_momentum_peak_Nms", "cmg_sigma_min",
       "qp_fail_count", "fallback_count", "max_constraint_violation",
       "solve_time_mean_ms"]


def write_table(case, arms_list):
    path = os.path.join(OUTROOT, case, "metrics.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["arm", "device", "label"] + KEY)
        for a in arms_list:
            p = os.path.join(OUTROOT, case, a, "metrics.json")
            if not os.path.exists(p):
                continue
            with open(p, encoding="utf-8") as f2:
                m = json.load(f2)
            w.writerow([a, m["device"], m["label"]]
                       + ["" if m.get(k) is None else m.get(k) for k in KEY])


def summary(case, arms_list):
    lines = ["# 装置替换实验（%s）\n" % case,
             "由 `scripts/run_device_sweep.py` 从各 arm 的 `metrics.json` 自动生成。\n",
             "| arm | 装置 | 三轴 RMSE [°] | 峰值 [°] | 滚转/俯仰/偏航 RMSE [°] | "
             "机动窗峰值 [°] | 机动后稳定 [s] | 舵面峰值 [°] | 饱和比例 | "
             "装置峰值力矩 [N·m] | 装置峰值角动量 [N·m·s] | σ_min | "
             "QP 失败 | 平均求解 [ms] |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for a in arms_list:
        p = os.path.join(OUTROOT, case, a, "metrics.json")
        if not os.path.exists(p):
            continue
        with open(p, encoding="utf-8") as f2:
            m = json.load(f2)

        def f(k, fmt="%.4g"):
            v = m.get(k)
            return "—" if v is None else (fmt % v if isinstance(v, float) else str(v))
        lines.append("| %s | %s | %s | %s | %.4g / %.4g / %.4g | %s | %s | %s | %s | %s | %s | %s | %s | %s |"
                     % (a, m["device"], f("err_rmse_deg"), f("err_peak_deg"),
                        m["err_rmse_roll_deg"], m["err_rmse_pitch_deg"],
                        m["err_rmse_yaw_deg"], f("peak_err_man_deg"),
                        f("maneuver_time_s"), f("delta_peak_all_deg"),
                        f("delta_sat_frac_all"), f("cmg_torque_peak_Nm"),
                        f("cmg_momentum_peak_Nms"), f("cmg_sigma_min"),
                        f("qp_fail_count"), f("solve_time_mean_ms")))
    lines.append("")
    p = os.path.join(OUTROOT, case, "summary.md")
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


# ---------------------------------------------------------------------------
def _parallel(jobs, nproc):
    import subprocess
    logdir = os.path.join(OUTROOT, "_logs")
    os.makedirs(logdir, exist_ok=True)
    pending, running, bad = list(jobs), [], []
    done = []
    while pending or running:
        while pending and len(running) < nproc:
            case, arm = pending.pop(0)
            logp = os.path.join(logdir, "%s_%s.log" % (case, arm))
            fh = open(logp, "w", encoding="utf-8")
            cmd = [sys.executable, os.path.abspath(__file__),
                   "--case", case, "--arms", arm]
            running.append((case, arm, subprocess.Popen(cmd, stdout=fh,
                                                        stderr=subprocess.STDOUT,
                                                        cwd=ROOT), fh, logp))
        time.sleep(1.0)
        for item in list(running):
            case, arm, proc, fh, logp = item
            if proc.poll() is None:
                continue
            fh.close()
            running.remove(item)
            mp = os.path.join(OUTROOT, case, arm, "metrics.json")
            if proc.returncode == 0 and os.path.exists(mp):
                with open(mp, encoding="utf-8") as f2:
                    done.append(json.load(f2))
                print("  done %s/%s" % (case, arm), flush=True)
            else:
                bad.append("%s/%s (rc=%s, log %s)" % (case, arm, proc.returncode, logp))
                print("  FAILED %s/%s, see %s" % (case, arm, logp), flush=True)
    return done, bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", nargs="*", default=None)
    ap.add_argument("--arms", nargs="*", default=None)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args(argv)

    _, blocks = load_sweep()
    cases = sorted(blocks["cases"])
    arm_map = blocks.get("arms", {})
    if a.list:
        for c in cases:
            print("%-10s %s" % (c, blocks["cases"][c].get("label", "")))
            print("           arms: %s" % ", ".join(arm_map.get(c, [])))
        print("\n已知 arm：")
        for k, v in arms.ARMS.items():
            print("  %-12s %s" % (k, v["label"]))
        return 0

    todo = cases if a.all else (a.case or ["cseries"])
    jobs = []
    for c in todo:
        if c not in arm_map:
            raise SystemExit("case %r has no arm list in %s" % (c, SWEEP))
        for arm in (a.arms or arm_map[c]):
            if arm not in arms.ARMS:
                raise SystemExit("unknown arm %r (see --list)" % arm)
            jobs.append((c, arm))
    print("cases:", todo, " arms:", a.arms or "(per case)", " jobs:", a.jobs)

    t0 = time.time()
    if a.jobs > 1 and len(jobs) > 1:
        _, bad = _parallel(jobs, a.jobs)
    else:
        bad = []
        for c, arm in jobs:
            run_one(c, arm)
    for c in todo:
        write_table(c, arm_map.get(c, []))
        summary(c, arm_map.get(c, []))
    print("\n%.0f s wall%s" % (time.time() - t0,
                               "" if not bad else ", FAILURES: " + "; ".join(bad)))
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
