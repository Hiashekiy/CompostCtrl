#!/usr/bin/env python
"""Automatic A/B configuration difference audit (implementation guide §4, §11).

For every scenario the resolved A and B configurations are compared key by key.
The only differences allowed are

    torque_gap.cmg_present, torque_gap.cmg_enabled

plus the model dimensions that follow necessarily from them (state / input /
NLP variable and constraint counts, gimbal limits that do not exist without a
CMG).  Any other difference is reported as a violation and makes the audit fail.

Outputs
    results/torque_gap_no_cmg_comparison/config_audit.md
    results/torque_gap_no_cmg_comparison/config_audit.json

python scripts/audit_no_cmg_configs.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, dump, read_yaml          # noqa: E402
from nearspace.torque_gap_ocp import TorqueGapOCP             # noqa: E402
from nearspace.torque_gap_scenario import Scenario            # noqa: E402

CFG = ROOT / "configs" / "torque_gap_no_cmg_comparison"
OUT = ROOT / "results" / "torque_gap_no_cmg_comparison"   # overridden by --out-root

ALLOWED_PATHS = {"torque_gap.cmg_present", "torque_gap.cmg_enabled"}
# Dimensions that differ because the physical system differs (never padded).
DERIVED = ("nx_model", "nu_model", "nlp_vars", "nlp_constraints")


def flat(obj, prefix=""):
    out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(flat(v, "%s.%s" % (prefix, k) if prefix else str(k)))
    elif isinstance(obj, list):
        out[prefix] = json.dumps(obj, sort_keys=True, default=str)
    else:
        out[prefix] = obj
    return out


def pairs():
    """[(scenario_id, A path, B path)] for every generated scenario."""
    out = []
    for sub in ("cases", "sequential"):
        for a in sorted((CFG / sub).glob("*_no_cmg.yaml")):
            b = a.with_name(a.name.replace("_no_cmg.yaml", "_with_cmg.yaml"))
            sid = a.name.replace("_no_cmg.yaml", "")
            out.append((sid, a, b, sub))
    return out


def describe(path, plant="P0"):
    cfg = _to_ns(read_yaml(str(path)))
    scn = Scenario(cfg)
    ocp = TorqueGapOCP(scn)
    return dict(nx_model=int(scn.nx), nu_model=int(scn.nu),
                nlp_vars=int(ocp.n_vars), nlp_constraints=int(ocp.n_cons),
                cmg_present=bool(scn.cmg_present),
                cmg_enabled=bool(scn.cmg_enabled),
                t_start=float(scn.t_start), t_end=float(scn.t_end),
                maneuvers=[dict(axis=m.axis, t0=m.t0, t1=m.t_end,
                                delta_deg=float(np.rad2deg(m.delta)), shape=m.shape)
                           for m in scn.traj.maneuvers],
                J=[float(scn.prm.veh.Jx), float(scn.prm.veh.Jy), float(scn.prm.veh.Jz)],
                aero=dict(S=float(scn.prm.veh.S), b=float(scn.prm.veh.b),
                          cbar=float(scn.prm.veh.cbar),
                          Cm_de=float(scn.prm.veh.Cm_de),
                          Cl_da=float(scn.prm.veh.Cl_da),
                          Cl_dr=float(scn.prm.veh.Cl_dr),
                          Cn_da=float(scn.prm.veh.Cn_da),
                          Cn_dr=float(scn.prm.veh.Cn_dr)),
                delta_max=dict(zip("de da dr".split(),
                                   [float(v) for v in np.rad2deg(scn.prm.veh.delta_max)])),
                delta_dot_max_deg_s=[float(v) for v in
                                     np.rad2deg(scn.prm.veh.delta_dot_max)],
                authority_scale=float(scn.auth.scale),
                weights=scn.w.as_dict(),
                disturbance=[float(v) for v in scn.traj.disturbance(120.0, cfg, scn.veh)],
                horizon_nodes=int(scn.rel_times.size),
                horizon_dts=[float(v) for v in scn.dts],
                dt_ctrl=float(scn.dt_ctrl), seed=int(getattr(scn, "seed", 0)),
                dist_phase=[float(v) for v in np.asarray(
                    getattr(scn, "dist_phase", np.zeros(3)), float)])


def main(argv=None):
    global OUT
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-root", default=None)
    args = ap.parse_args(argv)
    if args.out_root:
        OUT = ROOT / args.out_root if not Path(args.out_root).is_absolute() else Path(args.out_root)
        print("[audit] out-root =", OUT)
    OUT.mkdir(parents=True, exist_ok=True)
    rows, violations, audit = [], [], []
    for sid, a, b, sub in pairs():
        if not b.exists():
            violations.append((sid, "missing B file", str(b)))
            continue
        ca, cb = dump(_to_ns(read_yaml(str(a)))), dump(_to_ns(read_yaml(str(b))))
        da, db = describe(a), describe(b)
        fa, fb = flat(ca), flat(cb)
        diffs = []
        for key in sorted(set(fa) | set(fb)):
            va, vb = fa.get(key, "<absent>"), fb.get(key, "<absent>")
            if va != vb:
                diffs.append((key, va, vb))
                if key not in ALLOWED_PATHS:
                    violations.append((sid, key, "%r vs %r" % (va, vb)))
        derived = {k: (da[k], db[k]) for k in DERIVED}
        # shared-physics assertions (implementation guide §4, test #10)
        shared = dict(
            J_same=da["J"] == db["J"], aero_same=da["aero"] == db["aero"],
            deltas_same=da["delta_max"] == db["delta_max"],
            authority_same=da["authority_scale"] == db["authority_scale"],
            weights_same=da["weights"] == db["weights"],
            disturbance_same=da["disturbance"] == db["disturbance"],
            horizon_same=(da["horizon_nodes"] == db["horizon_nodes"]
                          and da["horizon_dts"] == db["horizon_dts"]),
            window_same=(da["t_start"], da["t_end"]) == (db["t_start"], db["t_end"]),
            maneuvers_same=da["maneuvers"] == db["maneuvers"],
            seed_same=(da["seed"], da["dist_phase"]) == (db["seed"], db["dist_phase"]),
        )
        for name, ok in shared.items():
            if not ok:
                violations.append((sid, "shared-physics " + name, "mismatch"))
        rows.append((sid, sub, diffs, derived, shared))
        audit.append(dict(scenario=sid, subdir=sub,
                          config_a=str(a.relative_to(ROOT)),
                          config_b=str(b.relative_to(ROOT)),
                          differing_keys=[d[0] for d in diffs],
                          derived_dims=derived, shared=shared,
                          A=da, B=db))

    lines = ["# A/B configuration audit — no-CMG vs surface+CMG",
             "",
             "Generated by `scripts/audit_no_cmg_configs.py`.  For every scenario the "
             "resolved A configuration (`cmg_present: false`) and B configuration "
             "(`cmg_present: true`) are compared key by key over the full inherited "
             "chain (vehicle, atmosphere, aerodynamic box, reference, solver, weights).",
             "",
             "Allowed differences: `torque_gap.cmg_present`, `torque_gap.cmg_enabled`, "
             "and the model dimensions that follow from them.  Any other difference "
             "is listed under **violations**.",
             "",
             "## Per-scenario result", "",
             "| scenario | dir | differing keys | NLP A (vars/cons) | NLP B (vars/cons) | verdict |",
             "|---|---|---|---|---|---|"]
    bad_by_scn = {v[0] for v in violations}
    for sid, sub, diffs, derived, shared in rows:
        keys = ", ".join("`%s`" % d[0] for d in diffs) or "—"
        lines.append("| %s | %s | %s | %d / %d | %d / %d | %s |" % (
            sid, sub, keys, derived["nlp_vars"][0], derived["nlp_constraints"][0],
            derived["nlp_vars"][1], derived["nlp_constraints"][1],
            "FAIL" if sid in bad_by_scn else "ok"))
    lines += ["", "## Shared physics (must be identical in both arms)", "",
              "| scenario | J | aero | delta limits | authority | weights | disturbance | horizon | window | manoeuvres | seed |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for sid, sub, diffs, derived, shared in rows:
        cells = ["ok" if shared[k] else "MISMATCH" for k in
                 ("J_same", "aero_same", "deltas_same", "authority_same",
                  "weights_same", "disturbance_same", "horizon_same",
                  "window_same", "maneuvers_same", "seed_same")]
        lines.append("| %s | %s |" % (sid, " | ".join(cells)))
    lines += ["", "## Model dimensions (differ by design)", "",
              "| scenario | nx A | nu A | nx B | nu B |",
              "|---|---|---|---|---|"]
    for sid, sub, diffs, derived, shared in rows:
        lines.append("| %s | %d | %d | %d | %d |" % (
            sid, derived["nx_model"][0], derived["nu_model"][0],
            derived["nx_model"][1], derived["nu_model"][1]))
    lines += ["", "## Violations", ""]
    if violations:
        lines += ["| scenario | key | detail |", "|---|---|---|"]
        lines += ["| %s | `%s` | %s |" % (v[0], v[1], v[2].replace("|", "\\|"))
                  for v in violations]
    else:
        lines.append("None.  The only A/B differences are `cmg_present`, "
                     "`cmg_enabled` and the resulting model dimensions.")
    (OUT / "config_audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (OUT / "config_audit.json").write_text(
        json.dumps(dict(violations=[list(v) for v in violations], scenarios=audit),
                   indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    print("scenarios audited: %d, violations: %d" % (len(rows), len(violations)))
    for v in violations:
        print("  VIOLATION", v)
    print("wrote", (OUT / "config_audit.md").relative_to(ROOT))
    return 1 if violations else 0


if __name__ == "__main__":
    sys.exit(main())
