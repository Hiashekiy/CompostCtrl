#!/usr/bin/env python
"""Closed-loop run of the torque-gap modulated joint optimisation.

Examples
--------
python scripts/run_torque_gap.py --case C1 --plant P0
python scripts/run_torque_gap.py --case C1 --plant P1
python scripts/run_torque_gap.py --case C2 --plant P1
python scripts/run_torque_gap.py --case C1 --set trajectory3.maneuver.delta_deg=15 --tag amp15

`--set a.b.c=value` applies a YAML-parsed override on the resolved config (used by
the ablation/intensity scans); every override is recorded in meta.json.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

CASES = {
    "C0": "c0_hold", "C1": "c1_pitch30", "C2": "c2_yaw30",
    "C3": "c3_pitch_then_roll", "C4": "c4_short_horizon", "C5": "c5_fixed_box",
}


def set_path(dst: dict, path: str, value):
    keys = path.split(".")
    node = dst
    for k in keys[:-1]:
        node = node.setdefault(k, {})
        if not isinstance(node, dict):
            raise SystemExit("override path %r is not a mapping" % path)
    node[keys[-1]] = value


def build_config(args):
    from nearspace.config import read_yaml, _to_ns
    name = CASES.get(args.case.upper(), args.case)
    path = Path(args.config_root) / ("%s.yaml" % name)
    if not path.is_absolute():
        path = ROOT / path
    if not path.exists():
        raise SystemExit("unknown case %r (%s)" % (args.case, path))
    raw = read_yaml(str(path))
    overrides = {}
    overrides.setdefault("torque_gap", {})["plant"] = args.plant.upper()
    overrides["torque_gap"]["seed"] = int(args.seed)
    if args.steps is not None:
        overrides["torque_gap"]["end"] = float(args.steps)
    for item in args.set or []:
        if "=" not in item:
            raise SystemExit("--set expects key=value, got %r" % item)
        key, val = item.split("=", 1)
        set_path(overrides, key, yaml.safe_load(val))
    from nearspace.config import deep_merge
    raw = deep_merge(raw, overrides)
    return _to_ns(raw), raw, str(path), overrides


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="C1")
    ap.add_argument("--plant", default="P0", help="P0 | P1")
    ap.add_argument("--seed", default=0, type=int)
    ap.add_argument("--steps", default=None, type=float, help="override t_end [s]")
    ap.add_argument("--set", action="append", default=None,
                    help="extra config override key=value (repeatable)")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=None, help="explicit output directory")
    ap.add_argument("--config-root", default="configs/torque_gap/cases",
                    help="directory holding the case YAMLs")
    ap.add_argument("--results-root", default="results/torque_gap",
                    help="root directory for the run output")
    args = ap.parse_args()

    from nearspace.config import dump
    from nearspace.torque_gap_ocp import TorqueGapOCP
    from nearspace.torque_gap_scenario import Scenario
    from nearspace.torque_gap_sim import ClosedLoop, write_json, write_run

    cfg, raw, cfg_path, overrides = build_config(args)
    scn = Scenario(cfg)
    scn.cfg_path = cfg_path
    ocp = TorqueGapOCP(scn)
    cl = ClosedLoop(scn, ocp=ocp, plant=args.plant)
    name = "%s_%s%s" % (args.case.upper(), args.plant.upper(),
                        ("_" + args.tag) if args.tag else "")
    root = Path(args.results_root)
    root = root if root.is_absolute() else ROOT / root
    out_dir = Path(args.out) if args.out else root / name
    os.makedirs(out_dir, exist_ok=True)
    print("[run] %s  plant=%s nodes=%d horizon=%.1fs steps=%d  (modulation removed)"
          % (name, cl.plant, ocp.N, scn.rel_times[-1], cl.n_steps))
    metrics = cl.run()
    meta = write_run(str(out_dir), cl, metrics, cfg,
                     extra_meta=dict(case=args.case.upper(),
                                     config_path=os.path.relpath(cfg_path, ROOT),
                                     overrides=overrides,
                                     command=" ".join([Path(sys.argv[0]).name] + sys.argv[1:])))
    (out_dir / "config_resolved.yaml").write_text(
        yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(json.dumps({k: metrics[k] for k in
                      ("rmse_deg", "max_err_deg", "fallback_ratio",
                       "cpu_s") if k in metrics}, indent=2))
    for k in ("migration_onset_t", "migration_lead_s", "settle_time_s",
              "gamma_at_maneuver_start", "nu_saturation_ratio_maneuver",
              "E_gap_maneuver_mean", "gap_residual_maneuver_mean", "H_norm_max"):
        if k in metrics:
            print("  %-32s %s" % (k, metrics[k]))
    print("[out] %s" % out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
