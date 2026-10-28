"""Phase 1 of the C1 reproduction study: freeze the environment and replay the
historical C1 (aerodynamic surfaces only) run.

    python scripts/reproduce_c1.py              # full: snapshot + run + verify
    python scripts/reproduce_c1.py --verify-only # re-verify existing artefacts

What it does
------------
1. Freezes the environment: git commit + `git status`, python/platform, the
   installed versions of the packages the run depends on, and the sha256 of
   every legacy source file and of `legacy/config.yaml`.  Written to
   ``results/c1_cmg_reproduction/baseline/meta.json``.
2. Copies the historical artefacts **as committed in git** (``git show
   HEAD:legacy/results/...``) into ``baseline/historical/`` so that a local
   re-run can never silently overwrite the record the report cites.
3. Runs ``legacy/run_all.py`` (which writes into ``legacy/results/``), copies
   the fresh C0-C4 outputs into ``baseline/rerun/``, and then restores
   ``legacy/results/`` from git so the tracked history stays untouched.
4. Compares the re-run C1 with the historical C1 against the acceptance rules
   fixed in advance (RMSE within 2 %, peak error within 2 %, identical QP
   failure count, per-step state differences reported numerically) and writes
   ``baseline/verification.json`` + ``baseline/verification.md``.

The acceptance numbers (RMSE 10.41 deg, peak 31.6 deg) are the historical
record, not a guarantee: the report must quote both the record and the value
actually obtained here.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

OUTROOT = os.path.join(ROOT, "results", "c1_cmg_reproduction")
BASE = os.path.join(OUTROOT, "baseline")

LEGACY = ["run_all.py", "model.py", "controllers.py", "mpc.py", "simulate.py",
          "config.yaml", "trajectory3_legacy.py"]
HIST_FILES = ["C1.csv", "metrics.csv", "C0.csv", "C2.csv", "C3.csv", "C4.csv"]

# acceptance rules, fixed before the run (implementation plan section 2.2)
RMSE_HIST_DEG = 10.413919510698577
PEAK_HIST_DEG = 31.585377771408016
RMSE_TOL_REL = 0.02
PEAK_TOL_REL = 0.02
RMS_LIMIT_DEG = 90.0          # a diverged record is not a reproduction


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def _git(*args):
    try:
        return subprocess.run(["git"] + list(args), cwd=ROOT, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                              ).stdout.decode("utf-8", "replace").strip()
    except Exception as exc:                       # no git: never fatal
        return "<git failed: %s>" % exc


def _read_metrics(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    out = {}
    for r in rows:
        out[r["name"].split()[0]] = r
    return out


def _float(d, key, default=float("nan")):
    try:
        return float(d[key])
    except (KeyError, TypeError, ValueError):
        return default


def _read_series(path):
    with open(path, newline="", encoding="utf-8") as fh:
        r = csv.DictReader(fh)
        cols = r.fieldnames
        arr = {c: [] for c in cols}
        for row in r:
            for c in cols:
                arr[c].append(float(row[c]))
    return {c: np.asarray(v) for c, v in arr.items()}


def high_window_km(cfg, km=90.0, cache=None):
    """Time interval during which the (frozen) skip profile is above `km`."""
    from nearspace.atmosphere_factory import make_atmosphere
    from nearspace.trajectory import SkipTrajectory
    atm = make_atmosphere(cfg, cache_dir=cache, backend=getattr(cfg.environment, "backend", "msis2"))
    traj = SkipTrajectory(cfg.trajectory, atm)
    t = np.arange(0.0, float(cfg.simulation.T) + 1e-9, float(cfg.simulation.dt_ctrl))
    h = np.array([traj.h(x) for x in t]) / 1000.0
    above = h >= km
    if not above.any():
        return None, t, h
    idx = np.flatnonzero(above)
    return (float(t[idx[0]]), float(t[idx[-1]])), t, h


# ---------------------------------------------------------------------------
# 1. environment snapshot
# ---------------------------------------------------------------------------
def freeze_environment():
    from importlib import metadata as md
    pkgs = {}
    for name in ("numpy", "scipy", "osqp", "pyyaml", "matplotlib", "pymsis"):
        try:
            pkgs[name] = md.version(name)
        except Exception:
            pkgs[name] = "<not installed>"
    snap = dict(
        git_commit=_git("rev-parse", "HEAD"),
        git_describe=_git("describe", "--always", "--dirty"),
        git_status_porcelain=_git("status", "--porcelain"),
        python=sys.version.split()[0],
        python_full=sys.version,
        platform=platform.platform(),
        executable=sys.executable,
        packages=pkgs,
        legacy_sources={f: _sha256(os.path.join(ROOT, "legacy", f))
                        for f in LEGACY if os.path.exists(os.path.join(ROOT, "legacy", f))},
    )
    return snap


# ---------------------------------------------------------------------------
# 2. historical artefacts out of git
# ---------------------------------------------------------------------------
def snapshot_history():
    d = os.path.join(BASE, "historical")
    os.makedirs(d, exist_ok=True)
    for f in HIST_FILES:
        blob = subprocess.run(["git", "show", "HEAD:legacy/results/" + f], cwd=ROOT,
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if blob.returncode != 0:
            print("  historical %s: not in git (%s)" % (f, blob.returncode))
            continue
        with open(os.path.join(d, f), "wb") as fh:
            fh.write(blob.stdout)
    return d


# ---------------------------------------------------------------------------
# 3. run the frozen legacy entry point
# ---------------------------------------------------------------------------
def run_legacy():
    logp = os.path.join(BASE, "run_all.log")
    env = dict(os.environ, PYTHONIOENCODING="utf-8", OMP_NUM_THREADS="1",
               OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    with open(logp, "w", encoding="utf-8") as fh:
        rc = subprocess.call([sys.executable, os.path.join("legacy", "run_all.py")],
                             cwd=ROOT, stdout=fh, stderr=subprocess.STDOUT, env=env)
    return rc, logp


def collect_rerun():
    d = os.path.join(BASE, "rerun")
    os.makedirs(d, exist_ok=True)
    for f in HIST_FILES:
        src = os.path.join(ROOT, "legacy", "results", f)
        if os.path.exists(src):
            with open(src, "rb") as fi, open(os.path.join(d, f), "wb") as fo:
                fo.write(fi.read())
    return d


def restore_history():
    """Put the tracked historical outputs back; the re-run lives in rerun/."""
    rc = subprocess.run(["git", "checkout", "--", "legacy/results"], cwd=ROOT,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
    dirty = _git("status", "--porcelain", "--", "legacy/results")
    return dict(checkout_rc=rc, still_dirty=dirty)


# ---------------------------------------------------------------------------
# 4. verification
# ---------------------------------------------------------------------------
def verify(hist_dir, rerun_dir):
    mh = _read_metrics(os.path.join(hist_dir, "metrics.csv"))
    mr = _read_metrics(os.path.join(rerun_dir, "metrics.csv"))
    h, r = mh["C1"], mr["C1"]

    checks = []

    def add(name, ok, value, target, note=""):
        checks.append(dict(check=name, ok=bool(ok), value=value, target=target, note=note))

    rmse_r = _float(r, "rmse_deg")
    rmse_h = _float(h, "rmse_deg")
    peak_r = _float(r, "max_err_deg")
    peak_h = _float(h, "max_err_deg")

    add("rmse_vs_history", abs(rmse_r / rmse_h - 1.0) <= RMSE_TOL_REL, rmse_r, rmse_h,
        "re-run RMSE within %.0f%% of the committed C1 RMSE" % (100 * RMSE_TOL_REL))
    add("rmse_vs_quoted", abs(rmse_r / RMSE_HIST_DEG - 1.0) <= RMSE_TOL_REL, rmse_r,
        RMSE_HIST_DEG, "re-run RMSE within %.0f%% of the 10.41 deg quoted in the plan"
        % (100 * RMSE_TOL_REL))
    add("peak_vs_history", abs(peak_r / peak_h - 1.0) <= PEAK_TOL_REL, peak_r, peak_h,
        "peak error within %.0f%% of the committed value" % (100 * PEAK_TOL_REL))
    add("peak_vs_quoted", abs(peak_r / PEAK_HIST_DEG - 1.0) <= PEAK_TOL_REL, peak_r,
        PEAK_HIST_DEG, "peak error within %.0f%% of the 31.6 deg quoted in the plan"
        % (100 * PEAK_TOL_REL))
    add("not_diverged", rmse_r < RMS_LIMIT_DEG, rmse_r, RMS_LIMIT_DEG,
        "a diverged record is never a reproduction")
    add("qp_fail_same", _float(r, "n_qp_fail") == _float(h, "n_qp_fail"),
        _float(r, "n_qp_fail"), _float(h, "n_qp_fail"), "QP failure count identical")
    add("sat_fraction_same", abs(_float(r, "sat_fraction") - _float(h, "sat_fraction")) <= 0.01,
        _float(r, "sat_fraction"), _float(h, "sat_fraction"), "saturation fraction within 1 pp")

    # point-by-point comparison of the recorded C1 timeline
    ch = _read_series(os.path.join(hist_dir, "C1.csv"))
    cr = _read_series(os.path.join(rerun_dir, "C1.csv"))
    diffs = {}
    n = min(len(ch["t"]), len(cr["t"]))
    for k in ("theta", "q", "delta", "theta_ref", "err_deg"):
        d = np.abs(ch[k][:n] - cr[k][:n])
        diffs[k] = dict(max_abs=float(d.max()), rms=float(np.sqrt(np.mean(d ** 2))))
    add("timeline_rows", len(ch["t"]) == len(cr["t"]), float(len(cr["t"])), float(len(ch["t"])),
        "same number of recorded steps")
    add("timeline_identical_to_1e-9", max(v["max_abs"] for v in diffs.values()) < 1e-9,
        max(v["max_abs"] for v in diffs.values()), 1e-9,
        "per-step states reproduce bit-for-bit if the environment is unchanged")

    keys = ("rmse_deg", "max_err_deg", "sat_fraction", "n_qp_fail", "eta_min")
    return dict(historical={k: _float(h, k) for k in keys},
                rerun={k: _float(r, k) for k in keys},
                checks=checks, timeline_diffs=diffs,
                passed=all(c["ok"] for c in checks))


def write_report(meta, ver):
    lines = ["# C1 reproduction verification", ""]
    lines.append("| item | value |")
    lines.append("|---|---|")
    for k in ("git_commit", "python", "platform", "executable"):
        lines.append("| %s | `%s` |" % (k, meta["environment"][k]))
    lines.append("| packages | %s |" % json.dumps(meta["environment"]["packages"]))
    lines.append("")
    lines.append("## metrics (committed history vs. re-run in this environment)")
    lines.append("")
    lines.append("| metric | historical | re-run | relative diff |")
    lines.append("|---|---|---|---|")
    for k in ver["historical"]:
        hh, rr = ver["historical"][k], ver["rerun"][k]
        rel = (rr / hh - 1.0) if hh else float("nan")
        lines.append("| %s | %.6f | %.6f | %+.3f %% |" % (k, hh, rr, 100 * rel))
    lines.append("")
    lines.append("## per-step timeline difference (re-run minus historical)")
    lines.append("")
    lines.append("| column | max abs | rms |")
    lines.append("|---|---|---|")
    for k, v in ver["timeline_diffs"].items():
        lines.append("| %s | %.3e | %.3e |" % (k, v["max_abs"], v["rms"]))
    lines.append("")
    lines.append("## acceptance checks (fixed before the run)")
    lines.append("")
    lines.append("| check | result | value | target | note |")
    lines.append("|---|---|---|---|---|")
    for c in ver["checks"]:
        lines.append("| %s | %s | %.6g | %.6g | %s |"
                     % (c["check"], "PASS" if c["ok"] else "FAIL", c["value"],
                        c["target"], c["note"]))
    lines.append("")
    lines.append("Verdict: **%s**" % ("PASSED" if ver["passed"] else "FAILED"))
    lines.append("")
    lines.append("The 10.41 deg / 31.6 deg pair quoted in the plan is the historical "
                 "record (10.4139 / 31.5854 in `legacy/results/metrics.csv`); the "
                 "numbers above are what this environment actually produced. The older "
                 "10.55 deg figure that circulates in the project notes is *not* "
                 "reproduced here and is not treated as evidence of a library-version "
                 "cause.")
    p = os.path.join(BASE, "verification.md")
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return p


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verify-only", action="store_true",
                    help="re-verify the artefacts already on disk")
    ap.add_argument("--skip-run", action="store_true", help="snapshot + verify only")
    a = ap.parse_args(argv)

    os.makedirs(BASE, exist_ok=True)
    meta_path = os.path.join(BASE, "meta.json")
    meta = {}
    if os.path.exists(meta_path) and a.verify_only:
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
    else:
        print("[1/4] freezing environment ...")
        meta = dict(environment=freeze_environment(), created_utc=None)

    print("[2/4] snapshotting the committed historical artefacts ...")
    hist_dir = snapshot_history()
    print("      ->", hist_dir)

    if not a.verify_only and not a.skip_run:
        print("[3/4] running legacy/run_all.py (this re-runs C0-C4, ~90 s) ...")
        rc, logp = run_legacy()
        print("      exit code %s, log %s" % (rc, logp))
        rerun_dir = collect_rerun()
        saved = restore_history()
        meta["rerun"] = dict(exit_code=rc, log=os.path.relpath(logp, ROOT), restore=saved)
    else:
        rerun_dir = os.path.join(BASE, "rerun")

    print("[4/4] verifying ...")
    ver = verify(hist_dir, rerun_dir)
    import time as _t
    meta.update(verification=ver, created_utc=meta.get("created_utc") or _t.strftime("%Y-%m-%dT%H:%M:%SZ"))

    # high-altitude window of the frozen profile: fixed here, reused by the A/B run
    from nearspace.config import load_config
    cfg = load_config(os.path.join(ROOT, "legacy", "config.yaml"))
    win, tt, hh = high_window_km(cfg, 90.0, cache=os.path.join(ROOT, "results", "_cache"))
    meta["frozen_profile"] = dict(
        high_altitude_threshold_km=90.0, high_altitude_window_s=win,
        h_max_km=float(hh.max()), h_at_window_peak_km=float(hh.max()),
        n_steps=int(len(tt)))
    print("      90 km window: %s (t_peak_h = %.1f s, h_max = %.1f km)"
          % (win, float(tt[int(np.argmax(hh))]), float(hh.max())))

    with open(meta_path, "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)
    p = write_report(meta, ver)
    for c in ver["checks"]:
        print("      %-26s %s  (%.6g vs %.6g)" % (c["check"], "PASS" if c["ok"] else "FAIL",
                                                  c["value"], c["target"]))
    print("\nverdict:", "PASSED" if ver["passed"] else "FAILED")
    print("wrote", meta_path)
    print("wrote", p)
    return 0 if ver["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
