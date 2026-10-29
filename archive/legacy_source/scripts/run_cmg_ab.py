"""Unified A/B run entry for the CMG cooperative-attitude-control study.

    python scripts/run_cmg_ab.py --case tracking --modes A B
    python scripts/run_cmg_ab.py --case maneuver10 maneuver20 --jobs 4
    python scripts/run_cmg_ab.py --all --jobs 4

The actuator mode is the ONLY thing that differs between A and B; see
configs/cmg_ab/matrix.yaml.  Every run writes, under
``results/cmg_ab/<case>/<mode>/``

    timeseries.csv          full state / actuator / solver record (§8.1)
    metrics.json            all §4.5 / §5.5 metrics
    meta.json               config snapshot, code version, initial state (§8.3)
    config_resolved.yaml    the effective configuration actually used
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import argparse
import json
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from nearspace import sim                                    # noqa: E402
from nearspace.config import _to_ns, deep_merge, dump, read_yaml   # noqa: E402
from nearspace.scenarios import (case_name, cmg_spec, controller_budgets,  # noqa: E402
                                 effective_weights, resolve)

MATRIX = os.path.join(ROOT, "configs", "cmg_ab", "matrix.yaml")
OUTROOT = os.path.join(ROOT, "results", "cmg_ab")

COLUMNS = [
    "t", "h", "V", "rho", "qinf",
    "phi_ref", "theta_ref", "psi_ref", "phi", "theta", "psi",
    "err_roll_deg", "err_pitch_deg", "err_yaw_deg", "err_norm_deg",
    "omega_x", "omega_y", "omega_z",
    "de_cmd", "da_cmd", "dr_cmd", "de", "da", "dr",
    "de_dot", "da_dot", "dr_dot",
    "M_aero_x", "M_aero_y", "M_aero_z",
    "gimbal_1", "gimbal_2", "gimbal_3", "gimbal_4",
    "gdot_1", "gdot_2", "gdot_3", "gdot_4",
    "H_cmg_x", "H_cmg_y", "H_cmg_z", "H_norm",
    "M_cmg_x", "M_cmg_y", "M_cmg_z", "sigma",
    "qp_ok", "qp_fail", "fallback", "qp_iter", "qp_converged", "viol",
    "solve_time_ms",
    "M_dist_x", "M_dist_y", "M_dist_z", "maneuver_active", "maneuver_phase",
]


# ---------------------------------------------------------------------------
# configuration resolution
# ---------------------------------------------------------------------------
def load_matrix(path=MATRIX):
    raw = read_yaml(path)
    blocks = {k: raw.pop(k, {}) for k in ("common", "modes", "cases")}
    return raw, blocks


def resolve_case(blocks, case):
    cases = blocks.get("cases", {})
    if case not in cases:
        raise SystemExit("unknown case %r; available: %s"
                         % (case, ", ".join(sorted(cases))))
    seen, cur = [], case
    while True:
        blk = cases[cur]
        seen.append(cur)
        parent = blk.get("extends")
        if not parent:
            break
        if parent not in cases or parent in seen:
            raise SystemExit("bad extends chain for %r" % case)
        cur = parent
    merged = {}
    for name in reversed(seen):
        merged = deep_merge(merged, cases[name])
    merged.pop("extends", None)
    return merged


def build_cfg(case, mode, path=MATRIX):
    base, blocks = load_matrix(path)
    if mode not in blocks.get("modes", {}):
        raise SystemExit("unknown mode %r" % mode)
    d = deep_merge(base, blocks.get("common", {}))
    d = deep_merge(d, resolve_case(blocks, case))
    d = deep_merge(d, blocks["modes"][mode])
    cfg = _to_ns(d)
    resolve(cfg)
    cfg.scenario.name = case
    cfg.scenario.mode_label = str(getattr(cfg, "label", mode))
    return cfg


def out_dir(case, mode, root=OUTROOT):
    return os.path.join(root, case, mode)


# ---------------------------------------------------------------------------
# one run
# ---------------------------------------------------------------------------
def _column_key(name):
    m = {
        "omega_x": "omega", "omega_y": "omega", "omega_z": "omega",
        "de_cmd": "delta_cmd_deg", "da_cmd": "delta_cmd_deg", "dr_cmd": "delta_cmd_deg",
        "de": "delta_deg", "da": "delta_deg", "dr": "delta_deg",
        "de_dot": "delta_dot_deg_s", "da_dot": "delta_dot_deg_s",
        "dr_dot": "delta_dot_deg_s",
        "M_aero_x": "M_aero", "M_aero_y": "M_aero", "M_aero_z": "M_aero",
        "gimbal_1": "gimbal_deg", "gimbal_2": "gimbal_deg", "gimbal_3": "gimbal_deg",
        "gimbal_4": "gimbal_deg",
        "gdot_1": "gdot", "gdot_2": "gdot", "gdot_3": "gdot", "gdot_4": "gdot",
        "H_cmg_x": "H_cmg", "H_cmg_y": "H_cmg", "H_cmg_z": "H_cmg",
        "M_cmg_x": "M_cmg", "M_cmg_y": "M_cmg", "M_cmg_z": "M_cmg",
        "M_dist_x": "M_dist", "M_dist_y": "M_dist", "M_dist_z": "M_dist",
    }
    return m.get(name, name)


def _column_index(name):
    groups = {"omega": ["omega_x", "omega_y", "omega_z"],
              "delta_cmd_deg": ["de_cmd", "da_cmd", "dr_cmd"],
              "delta_deg": ["de", "da", "dr"],
              "delta_dot_deg_s": ["de_dot", "da_dot", "dr_dot"],
              "M_aero": ["M_aero_x", "M_aero_y", "M_aero_z"],
              "gimbal_deg": ["gimbal_1", "gimbal_2", "gimbal_3", "gimbal_4"],
              "gdot": ["gdot_1", "gdot_2", "gdot_3", "gdot_4"],
              "H_cmg": ["H_cmg_x", "H_cmg_y", "H_cmg_z"],
              "M_cmg": ["M_cmg_x", "M_cmg_y", "M_cmg_z"],
              "M_dist": ["M_dist_x", "M_dist_y", "M_dist_z"]}
    key = _column_key(name)
    idx = groups.get(key, [name]).index(name)
    return key, idx


def write_timeseries(path, rec):
    cols = []
    for name in COLUMNS:
        key, idx = _column_index(name)
        a = np.asarray(rec[key], dtype=float)
        cols.append(a if a.ndim == 1 else a[:, idx])
    arr = np.column_stack(cols)
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(",".join(COLUMNS) + "\n")
        for row in arr:
            fh.write(",".join("%.10g" % v for v in row) + "\n")


def run_one(case, mode, verbose=True, tag=None, config_path=MATRIX,
            outroot=OUTROOT, cache_dir=None):
    t0 = time.time()
    cfg = build_cfg(case, mode, config_path)
    od = out_dir(case, mode, outroot)
    os.makedirs(od, exist_ok=True)

    if verbose:
        from nearspace.model6c import VehicleCMGParams
        prm_chk = VehicleCMGParams.from_config(cfg)
        print("=" * 78)
        print("case=%s  mode=%s  %s" % (case, mode, cfg.scenario.mode_label))
        print("  weights in force :", json.dumps(effective_weights(cfg), ensure_ascii=False))
        print("  controller budget:", json.dumps(controller_budgets(cfg, prm_chk), ensure_ascii=False))
        print("  CMG hardware     :", json.dumps(cmg_spec(cfg, prm_chk), ensure_ascii=False))
        print("  horizon          : T=%.0f s  dt_ctrl=%.3f  dt_plant=%.4f"
              % (cfg.simulation.T, cfg.simulation.dt_ctrl, cfg.simulation.dt_plant))
        print("=" * 78, flush=True)

    rec = sim.run(cfg, cache_dir=cache_dir or os.path.join(OUTROOT, "_cache"),
                  progress=verbose)
    write_timeseries(os.path.join(od, "timeseries.csv"), rec)
    with open(os.path.join(od, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(rec["metrics"], fh, indent=2, ensure_ascii=False, default=_j)
    meta = sim.metadata(cfg, rec, case, tag or time.strftime("%Y%m%dT%H%M%S"),
                        extra=dict(label=cfg.scenario.mode_label,
                                   wall_time_breakdown_s=time.time() - t0))
    with open(os.path.join(od, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False, default=_j)
    with open(os.path.join(od, "config_resolved.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(dump(cfg), fh, allow_unicode=True, sort_keys=False)
    m = rec["metrics"]
    if verbose:
        print("[%s/%s] RMSE=%.4f deg  peak=%.3f deg  |H|max=%.1f  sigma_min=%.2f  "
              "QP fail=%d  fallback=%d  wall=%.0fs"
              % (case, mode, m["err_rmse_deg"], m["err_peak_deg"],
                 m["cmg_momentum_peak_Nms"], m["cmg_sigma_min"],
                 m["qp_fail_count"], m["fallback_count"], rec["wall_time_s"]),
              flush=True)
    return dict(case=case, mode=mode, metrics=m, outdir=od)


def _j(o):
    if hasattr(o, "tolist"):
        return o.tolist()
    return str(o)


# ---------------------------------------------------------------------------
# metrics table
# ---------------------------------------------------------------------------
def update_tables(case_metrics, outroot=OUTROOT):
    """Append one row per (case, mode) to <outroot>/<case>/metrics.csv."""
    fields = ["case", "mode", "err_rmse_deg", "err_peak_deg",
              "err_rmse_roll_deg", "err_rmse_pitch_deg", "err_rmse_yaw_deg",
              "peak_err_dist_deg", "peak_err_dist_tail_deg",
              "recovery_time_dist_s", "recovery_time_dist_rel_s",
              "peak_err_man_deg", "maneuver_time_s", "maneuver_completed",
              "hold_rmse_deg", "max_overshoot_pitch_deg",
              "delta_peak_all_deg", "delta_sat_frac_all", "delta_travel_deg",
              "cmg_torque_peak_Nm", "cmg_momentum_peak_Nms",
              "cmg_gimbal_rate_peak_rad_s", "cmg_sigma_min",
              "qp_fail_count", "qp_fail_longest_run", "qp_not_converged_count",
              "qp_iter_mean", "fallback_count",
              "max_constraint_violation", "solve_time_mean_ms", "solve_time_p95_ms",
              "wall_time_s"]
    for r in case_metrics:
        d = os.path.join(outroot, r["case"])
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "metrics.csv")
        rows = []
        if os.path.exists(path):
            import csv
            with open(path, newline="", encoding="utf-8") as fh:
                rows = [row for row in csv.DictReader(fh)
                        if row.get("mode") != r["mode"]]
        row = dict(case=r["case"], mode=r["mode"])
        for f in fields[2:]:
            v = r["metrics"].get(f)
            row[f] = "" if v is None else v
        rows.append(row)
        rows.sort(key=lambda x: x["mode"])
        import csv
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            for rr in rows:
                w.writerow({f: rr.get(f, "") for f in fields})


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _worker(args):
    case, mode = args
    try:
        return run_one(case, mode, verbose=False)
    except Exception as exc:                     # keep the batch alive
        import traceback
        traceback.print_exc()
        return dict(case=case, mode=mode, error=str(exc), metrics={})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", nargs="*", default=None)
    ap.add_argument("--modes", nargs="*", default=["A", "B"])
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--config", default=MATRIX,
                    help="experiment YAML in the base/common/modes/cases form")
    ap.add_argument("--out", default=OUTROOT, help="results root for this experiment")
    a = ap.parse_args(argv)
    matrix, outroot = os.path.abspath(a.config), os.path.abspath(a.out)

    _, blocks = load_matrix(matrix)
    cases = sorted(blocks.get("cases", {}))
    if a.list:
        for c in cases:
            print("%-18s %s" % (c, blocks["cases"][c].get("label", "")))
        return 0
    todo = cases if a.all else (a.case or ["tracking"])

    os.makedirs(outroot, exist_ok=True)
    print("config:", matrix)
    print("cases:", todo, " modes:", a.modes, " jobs:", a.jobs)

    # warm the NRLMSIS-2.0 cache once, serially, so parallel workers only read
    _warm_cache(todo[0], a.modes[0], matrix, outroot)

    jobs = [(c, m) for c in todo for m in a.modes]
    t0 = time.time()
    if a.jobs > 1 and len(jobs) > 1:
        results, bad = _parallel(jobs, a.jobs, a.tag, matrix, outroot)
    else:
        results = [run_one(c, m, verbose=True, tag=a.tag, config_path=matrix,
                           outroot=outroot) for c, m in jobs]
        bad = []
    ok = [r for r in results if r and "error" not in r]
    update_tables(ok, outroot)
    print("\n%d runs ok, %d failed, %.0f s wall" % (len(ok), len(bad), time.time() - t0))
    for r in bad:
        print("  FAILED", r)
    return 0 if not bad else 1


def _parallel(jobs, nproc, tag, matrix=MATRIX, outroot=OUTROOT):
    """Run each (case, mode) in its own process.

    Windows named pipes are unavailable in this environment, so multiprocessing
    /ProcessPoolExecutor cannot be used; children are spawned with their stdio
    redirected to log FILES (never a pipe) and the result is read back from the
    artefacts they write.
    """
    import subprocess
    logdir = os.path.join(outroot, "_logs")
    os.makedirs(logdir, exist_ok=True)
    pending = list(jobs)
    running = []
    done, bad = [], []
    while pending or running:
        while pending and len(running) < nproc:
            case, mode = pending.pop(0)
            logp = os.path.join(logdir, "%s_%s.log" % (case, mode))
            fh = open(logp, "w", encoding="utf-8")
            cmd = [sys.executable, os.path.abspath(__file__),
                   "--case", case, "--modes", mode, "--jobs", "1",
                   "--config", matrix, "--out", outroot]
            if tag:
                cmd += ["--tag", tag]
            running.append((case, mode, subprocess.Popen(cmd, stdout=fh,
                                                         stderr=subprocess.STDOUT,
                                                         cwd=ROOT), fh, logp))
        time.sleep(1.0)
        for item in list(running):
            case, mode, proc, fh, logp = item
            if proc.poll() is None:
                continue
            fh.close()
            running.remove(item)
            mp = os.path.join(outroot, case, mode, "metrics.json")
            if proc.returncode == 0 and os.path.exists(mp):
                with open(mp, encoding="utf-8") as f2:
                    done.append(dict(case=case, mode=mode, metrics=json.load(f2)))
                print("  done %s/%s" % (case, mode), flush=True)
            else:
                bad.append("%s/%s (rc=%s, log %s)" % (case, mode, proc.returncode, logp))
                print("  FAILED %s/%s, see %s" % (case, mode, logp), flush=True)
    return done, bad


def _warm_cache(case, mode, matrix=MATRIX, outroot=OUTROOT):
    try:
        cfg = build_cfg(case, mode, matrix)
        from nearspace.atmosphere_factory import make_atmosphere
        make_atmosphere(cfg, cache_dir=os.path.join(OUTROOT, "_cache"),
                        backend=getattr(cfg.environment, "backend", "msis2"))
    except Exception as exc:
        print("cache warm-up skipped:", exc)


if __name__ == "__main__":
    raise SystemExit(main())
