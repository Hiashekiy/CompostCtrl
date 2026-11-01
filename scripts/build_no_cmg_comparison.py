#!/usr/bin/env python
"""Generate the no-CMG / with-CMG A/B configuration set.

One scenario file per case carries the *only* description of the manoeuvre and
of the shared physics; the A and B case files are derived from it, so the two
arms cannot drift apart through hand-editing (implementation guide §10).

    configs/torque_gap_no_cmg_comparison/
        base.yaml                 common physics, inherited from the M50 C1 chain
        scenarios/<ID>.yaml       manoeuvre + window of one scenario
        cases/<ID>_no_cmg.yaml    A: cmg_present: false, cmg_enabled: false
        cases/<ID>_with_cmg.yaml  B: cmg_present: true,  cmg_enabled: true
        sequential/<ID>_no_cmg.yaml / _with_cmg.yaml

python scripts/build_no_cmg_comparison.py            # write the files
python scripts/build_no_cmg_comparison.py --check    # fail if they are stale
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "configs" / "torque_gap_no_cmg_comparison"
PARENT_CASE = "../torque_gap_m50/cases/c1_pitch30.yaml"   # M50 C1 chain

# id, title, manoeuvres, t_end, sub-directory
SCENARIOS = [
    dict(id="S00_pitch30", title="Pitch +30 deg, hold", t_end=150.0, sub="cases",
         maneuvers=[dict(axis="pitch", start=110.0, duration=20.0, delta_deg=30.0,
                         shape="hold")]),
    dict(id="S01_pitch_minus30", title="Pitch -30 deg, hold", t_end=150.0, sub="cases",
         maneuvers=[dict(axis="pitch", start=110.0, duration=20.0, delta_deg=-30.0,
                         shape="hold")]),
    dict(id="S02_pitch10", title="Pitch +10 deg, hold", t_end=150.0, sub="cases",
         maneuvers=[dict(axis="pitch", start=110.0, duration=20.0, delta_deg=10.0,
                         shape="hold")]),
    dict(id="S03_pitch45", title="Pitch +45 deg, hold", t_end=150.0, sub="cases",
         maneuvers=[dict(axis="pitch", start=110.0, duration=20.0, delta_deg=45.0,
                         shape="hold")]),
    dict(id="S04_roll30", title="Roll +30 deg, hold", t_end=150.0, sub="cases",
         maneuvers=[dict(axis="roll", start=110.0, duration=20.0, delta_deg=30.0,
                         shape="hold")]),
    dict(id="S05_yaw30", title="Yaw +30 deg, hold", t_end=150.0, sub="cases",
         maneuvers=[dict(axis="yaw", start=110.0, duration=20.0, delta_deg=30.0,
                         shape="hold")]),
    dict(id="S06_pitch30_roll20", title="Pitch +30 deg and Roll +20 deg together, hold",
         t_end=150.0, sub="cases",
         maneuvers=[dict(axis="pitch", start=110.0, duration=20.0, delta_deg=30.0,
                         shape="hold"),
                    dict(axis="roll", start=110.0, duration=20.0, delta_deg=20.0,
                         shape="hold")]),
    dict(id="S07_sequential", title="Pitch +30 deg then Pitch -30 deg", t_end=180.0,
         sub="sequential",
         maneuvers=[dict(axis="pitch", start=110.0, duration=20.0, delta_deg=30.0,
                         shape="hold"),
                    dict(axis="pitch", start=150.0, duration=20.0, delta_deg=-30.0,
                         shape="hold")]),
]

BASE_YAML = """\
# Common physics of the no-CMG (A) vs surface+CMG (B) comparison.
# ---------------------------------------------------------------- inherited --
# Everything about the vehicle, its inertia, the aerodynamic coefficients, the
# aero authority box, the gimbal device (M50 published limits), the reference
# trajectory, the disturbance, the prediction grid, the solver and the tracking
# weights is inherited unchanged from the M50 C1 chain.  This experiment only
# adds the CMG presence switch and the scenario manoeuvre.  Nothing here changes
# a run that does not load this file.
base: {parent}

torque_gap:
  start: 90.0
  end: 150.0
  dt_ctrl: 0.2
  plant: P0
  plant_substeps: 8
  seed: 0
  # The baseline disturbance of the reference trajectory is deterministic, so
  # `seed` alone would change nothing.  `seed_randomize: true` draws a phase
  # offset for its sinusoidal components; both arms of a scenario always share
  # one seed, hence one identical external torque history.
  seed_randomize: false
  # B group defaults.  The A-group case files override both to false; the
  # configuration check rejects cmg_present: false with cmg_enabled: true.
  cmg_present: true
  cmg_enabled: true
  gamma_limit: none          # unwrapped gimbal angle (M50 chain)
"""

SCENARIO_HEADER = """\
# {title}  --  {sid}
# The single definition of this scenario: window {t0:.0f}-{t1:.0f} s and the
# reference manoeuvre(s).  The A/B case files are generated from this file, so
# both arms share the physics byte for byte.
base: {base}
torque_gap:
  start: {t0:.1f}
  end: {t1:.1f}
trajectory3:
  maneuver:
{man}
"""

CASE_HEADER = """\
# {arm} arm of {sid} ({title}).
{what}
# Everything else is inherited from scenarios/{sid}.yaml; see
# scripts/audit_no_cmg_configs.py for the automatic A/B difference audit.
base: {base}
torque_gap:
  cmg_present: {present}
  cmg_enabled: {enabled}
"""

ARM_A = ("# A group: the vehicle carries NO CMG.  There is no gimbal state, no gimbal\n"
         "# rate input, no CMG torque and no rotor-momentum gyroscopic coupling in\n"
         "# either the optimiser or the truth plant (7 states / 3 inputs).")
ARM_B = ("# B group: the vehicle carries the 4-unit pyramid CMG and the surfaces and\n"
         "# the CMG are optimised jointly (11 states / 7 inputs).")


def _man_block(maneuvers):
    if len(maneuvers) == 1:
        m = maneuvers[0]
        return "    axis: %s\n    start: %.1f\n    duration: %.1f\n    delta_deg: %.1f\n    shape: %s" % (
            m["axis"], m["start"], m["duration"], m["delta_deg"], m["shape"])
    lines = []
    for m in maneuvers:
        lines.append("    - {axis: %s, start: %.1f, duration: %.1f, delta_deg: %.1f, shape: %s}"
                     % (m["axis"], m["start"], m["duration"], m["delta_deg"], m["shape"]))
    return "\n".join(lines)


def files():
    """Map of relative path -> file content for the whole generated config set."""
    out = {"base.yaml": BASE_YAML.format(parent=PARENT_CASE)}
    for s in SCENARIOS:
        sid, sub = s["id"], s["sub"]
        scenario_path = "scenarios/%s.yaml" % sid
        out[scenario_path] = SCENARIO_HEADER.format(
            title=s["title"], sid=sid, base="../base.yaml",
            t0=90.0, t1=s["t_end"], man=_man_block(s["maneuvers"]))
        for arm, present, enabled, what in (("no_cmg", "false", "false", ARM_A),
                                            ("with_cmg", "true", "true", ARM_B)):
            name = "%s_%s.yaml" % (sid, arm)
            out["%s/%s" % (sub, name)] = CASE_HEADER.format(
                arm="A" if arm == "no_cmg" else "B", sid=sid, title=s["title"],
                what=what, base="../scenarios/%s.yaml" % sid,
                present=present, enabled=enabled)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="do not write; fail when a file is missing or stale")
    args = ap.parse_args()
    CFG.mkdir(parents=True, exist_ok=True)
    stale = []
    for rel, text in sorted(files().items()):
        path = CFG / rel
        if path.exists() and path.read_text(encoding="utf-8") == text:
            continue
        if args.check:
            stale.append(rel)
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        print("wrote", path.relative_to(ROOT))
    if args.check:
        for rel in stale:
            print("STALE", rel)
        return 1 if stale else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
