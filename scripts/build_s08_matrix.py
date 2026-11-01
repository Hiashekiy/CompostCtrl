#!/usr/bin/env python
"""Generate the S08 scenario / A-B case configuration files.

    python scripts/build_s08_matrix.py            # (re)write the files
    python scripts/build_s08_matrix.py --check    # fail if the files drifted

Generated tree
    configs/torque_gap_s08/scenarios/<case>.yaml        single definition of a case
    configs/torque_gap_s08/cases/<case>_{no_cmg,with_cmg}.yaml   the two arms

A case file only defines the *flight condition* (angle of attack and dynamic
pressure) of the controlled experiments; the two arm files only switch the CMG
presence.  Nothing about the vehicle, the aerodynamics, the disturbance, the
solver or the weights is written here: all of that comes from the frozen base
chain (see configs/torque_gap_s08/base.yaml).

How the flight condition is realised (see docs/torque_gap/S08_code_audit.md)
    angle of attack   alpha_ref = theta_ref - gamma = alpha_trim + manoeuvre.
                      With a level, constant-condition trajectory (gamma = 0 and
                      no manoeuvre) alpha_ref is exactly `trajectory.alpha_trim_deg`
                      for the whole window.
    dynamic pressure  qbar = 0.5 rho(h0) V0^2 with rho from the MSIS2 backend.
                      A2/A3 keep the airspeed of the original S00 evaluation
                      window and solve h0 so that qbar hits the target, i.e. the
                      qbar is changed the way the original descent changes it
                      (density), leaving Mach nearly constant.
    reference level   Q0 = qbar of the original S00/S03 trajectory at t = 90 s
                      (the start of the 90-150 s evaluation window), V_ref = its
                      airspeed at the same instant.

Everything is deterministic: the numbers written into the YAML files are
recomputed from the frozen atmosphere by this script.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.atmosphere_factory import make_atmosphere     # noqa: E402
from nearspace.config import load_config                     # noqa: E402
from nearspace.trajectory3 import Trajectory3D               # noqa: E402

CFG = ROOT / "configs" / "torque_gap_s08"
OUT = ROOT / "results" / "torque_gap_s08"
REF_CASE = "configs/torque_gap_m50/cases/c1_pitch30.yaml"
T_REF = 90.0            # start of the common evaluation window

# Doc §5 / §6 scan points.  A1: an angle-of-attack sweep at one fixed qbar.
ALPHA_SWEEP = [-10.0, 0.0, 10.0, 20.0, 30.0, 40.0]
# A2: a dynamic-pressure sweep at the trim angle of attack of the original
# trajectory (2 deg), i.e. without changing the reference angle of attack.
QBAR_FACTORS = [0.5, 1.0, 2.0, 4.0]
ALPHA_TRIM_REF = 2.0
# A3: the 2 x 2 combination.  The low/high levels are chosen from the A1/A2
# results (they are the extremes that stay inside the model's own behaviour) and
# recorded in results/torque_gap_s08/summary.md.
A3_ALPHA = (0.0, 30.0)
A3_QBAR_FACTOR = (0.5, 2.0)
# Every controlled case (A1-A3) flies the *same* pitch manoeuvre superimposed on
# its own trim angle of attack: a raised-cosine bump of +BUMP_DEG centred in the
# window (110-130 s), which returns to the trim value at both ends.  The
# inter-case difference is therefore still exactly the alpha / qbar level, while
# every case exercises a real transient (the "capture" phase) instead of a
# perfectly steady hold, so the sweep measures control load and not only
# feasibility.  Identical in A and B: the manoeuvre lives in the scenario file.
BUMP_DEG = 8.0
BUMP_START = 110.0
BUMP_DUR = 20.0


# --------------------------------------------------------------------------
# reference flight condition of the original descent trajectory
# --------------------------------------------------------------------------
def reference_condition():
    cfg = load_config(str(ROOT / REF_CASE))
    atm = make_atmosphere(cfg)
    traj = Trajectory3D(cfg, atm)
    V = float(traj.V(T_REF))
    rho = float(traj.rho(T_REF))
    return dict(t=T_REF, V=V, rho=rho, qbar=0.5 * rho * V ** 2,
                h_m=float(traj.h(T_REF)),
                alpha_trim_deg=float(cfg.trajectory.alpha_trim_deg),
                cbar=float(cfg.vehicle3.cbar),
                # qbar at the t = 0 point of the original trajectory.  This is the
                # reference at which `vehicle3.Md_amp` is defined and by which the
                # disturbance model scales the aerodynamic part
                # (trajectory3.dist_scale_qinf uses qinf(t)/qinf(0)).
                qbar_ref0=float(traj.qinf(0.0)),
                Md_amp=np.array(cfg.vehicle3.Md_amp, dtype=float),
                atm=atm)


def qbar_of(h_m, V, atm):
    return 0.5 * float(atm.rho(float(h_m))) * V ** 2


def solve_h0(target_qbar, V, atm, lo_km=30.0, hi_km=110.0):
    """Altitude whose density gives the target qbar at the given airspeed."""
    f = lambda h: np.log(qbar_of(h, V, atm)) - np.log(target_qbar)   # noqa: E731
    h = brentq(f, lo_km * 1000.0, hi_km * 1000.0, xtol=1e-3, rtol=1e-12)
    return float(h)


# --------------------------------------------------------------------------
# case definitions
# --------------------------------------------------------------------------
def cases(ref):
    """List of case dicts: name, group, description, flight condition."""
    out = []
    for a in ALPHA_SWEEP:
        out.append(dict(
            name="A1_alpha_%s" % _tag(a), group="A1_alpha",
            title="S08-A1 angle-of-attack sweep: alpha_ref = %+.0f deg at fixed qbar" % a,
            note=("Fixed dynamic pressure (qbar = %.2f Pa = Q0), level flight, "
                  "constant airspeed.  Only the reference angle of attack "
                  "changes." % ref["qbar"]),
            alpha_deg=float(a), qbar_factor=1.0))
    for f in QBAR_FACTORS:
        out.append(dict(
            name="A2_qbar_%s" % _tag(f), group="A2_qbar",
            title="S08-A2 dynamic-pressure sweep: qbar = %.2f x Q0 at fixed alpha" % f,
            note=("Fixed reference angle of attack (alpha_ref = %.1f deg, the trim "
                  "angle of the original trajectory).  qbar is changed by density "
                  "(altitude) at the airspeed of the original evaluation window, "
                  "so the Mach number stays nearly constant."
                  % ALPHA_TRIM_REF),
            alpha_deg=ALPHA_TRIM_REF, qbar_factor=float(f)))
    for fa, fq in ((0, 0), (1, 0), (0, 1), (1, 1)):
        out.append(dict(
            name="A3_coupled_%s" % ("lh"[fa] + "lh"[fq]), group="A3_coupled",
            title=("S08-A3 coupled case: alpha = %+.0f deg, qbar = %.2f x Q0"
                   % (A3_ALPHA[fa], A3_QBAR_FACTOR[fq])),
            note=("2 x 2 factor combination of the A1/A2 levels: alpha = %+.0f deg, "
                  "qbar = %.2f x Q0." % (A3_ALPHA[fa], A3_QBAR_FACTOR[fq])),
            alpha_deg=float(A3_ALPHA[fa]),
            qbar_factor=float(A3_QBAR_FACTOR[fq])))
    for tag, deg in (("S00", 30.0), ("S03", 45.0)):
        out.append(dict(
            name="A4_descent_replay_%s" % tag, group="A4_descent_replay",
            title=("S08-A4 descent replay: original %s trajectory and +%.0f deg "
                   "pitch reference" % (tag, deg)),
            note=("Original skip trajectory, original 90-150 s evaluation window, "
                  "original +%.0f deg pitch manoeuvre (110-130 s).  Nothing but "
                  "the CMG presence distinguishes the two arms." % deg),
            replay=True, maneuver_deg=deg))
    return out


def _tag(value):
    """File-name-safe tag for a numeric scan point (+30 -> p30, -10 -> m10)."""
    v = float(value)
    s = "%+d" % int(round(v)) if abs(v - round(v)) < 1e-9 else "%+.1f" % v
    return s.replace("+", "p").replace("-", "m").replace(".", "p")


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------
def render_scenario(case, ref):
    head = [
        "# %s" % case["title"],
        "# %s" % case["note"],
        "#",
        "# Generated by scripts/build_s08_matrix.py -- do not edit by hand.",
        "# The A/B arm files (cases/%s_{no_cmg,with_cmg}.yaml) are generated from" % case["name"],
        "# this file, so both arms share the flight condition byte for byte.",
        "base: ../base.yaml",
        "torque_gap:",
        "  start: %.1f" % 90.0,
        "  end: %.1f" % 150.0,
    ]
    if case.get("replay"):
        body = [
            "trajectory3:",
            "  maneuver:",
            "    axis: pitch",
            "    start: 110.0",
            "    duration: 20.0",
            "    delta_deg: %.1f" % case["maneuver_deg"],
            "    shape: hold",
        ]
        return "\n".join(head + body) + "\n"
    fc = case["flight"]
    scale = fc["qbar_achieved"] / ref["qbar_ref0"]
    md_note = ("  # qbar_case / qbar(t=0) = %.3e, i.e. the effective amplitude is "
               "%s N.m x qbar/%.1f Pa; the q-independent residual Md0 is unchanged."
               % (scale, np.round(ref["Md_amp"], 3).tolist(), ref["qbar_ref0"]))
    body = [
        "trajectory:",
        "  # Constant flight condition.  amp = 0 keeps the altitude (hence the",
        "  # density and qbar) constant and makes gamma = 0; dVdt = 0 keeps the",
        "  # airspeed (hence the Mach number) constant.",
        "  h0: %.1f" % fc["h_m"],
        "  amp: 0.0",
        "  period: 240.0",
        "  V0: %.1f" % fc["V"],
        "  dVdt: 0.0",
        "  # alpha_ref = theta_ref - gamma = alpha_trim + manoeuvre; with gamma = 0",
        "  # and no manoeuvre the reference angle of attack is exactly this value.",
        "  alpha_trim_deg: %.1f" % fc["alpha_deg"],
        "  maneuver_amp_deg: 0.0",
        "  maneuver_start: 1.0e9",
        "  maneuver_duration: 10.0",
        "vehicle3:",
        "  # The aerodynamic part of the external disturbance (Md_amp) is defined at",
        "  # the 30 km / 4000 m/s reference condition of the original trajectory and",
        "  # scales with qbar there (trajectory3.dist_scale_qinf normalises by",
        "  # qinf(0) of the case's own trajectory, which for this constant-condition",
        "  # case is the case's own qbar).  To reproduce the *physical* disturbance",
        "  # effective amplitude equals the original law at this qbar:",
        md_note,
        "  Md_amp: [%.6g, %.6g, %.6g]" % tuple(fc["Md_amp"]),
        "trajectory3:",
        "  # The same +%.1f deg pitch bump for every controlled case:" % BUMP_DEG,
        "  # alpha_ref rises from the case level to level + %.1f deg over %.0f-%.0f s"
        % (BUMP_DEG, BUMP_START, BUMP_START + BUMP_DUR),
        "  # and returns to the case level, so the case keeps its nominal alpha while",
        "  # the transient exposes the control load (surface moment, CMG load).",
        "  maneuver:",
        "    axis: pitch",
        "    start: %.1f" % BUMP_START,
        "    duration: %.1f" % BUMP_DUR,
        "    delta_deg: %.1f" % BUMP_DEG,
        "    shape: bump",
    ]
    return "\n".join(head + body) + "\n"


def render_arm(case, arm):
    if arm == "no_cmg":
        head = [
            "# A arm of %s -- aerodynamic surfaces only, no CMG at all." % case["name"],
            "# The vehicle carries no rotor momentum, no gimbal state and no gimbal",
            "# rate; the optimiser is a 7-state / 3-input problem (doc §10).",
            "base: ../scenarios/%s.yaml" % case["name"],
            "torque_gap:",
            "  cmg_present: false",
            "  cmg_enabled: false",
        ]
    else:
        head = [
            "# B arm of %s -- surfaces + the current 4-unit pyramid CMG." % case["name"],
            "# 11 states / 7 inputs, M50 gimbal limits, joint optimisation.",
            "base: ../scenarios/%s.yaml" % case["name"],
            "torque_gap:",
            "  cmg_present: true",
            "  cmg_enabled: true",
        ]
    return "\n".join(head) + "\n"


# --------------------------------------------------------------------------
def build():
    ref = reference_condition()
    atm = ref["atm"]
    out = {}
    design = dict(
        reference=dict(t_s=ref["t"], h_m=ref["h_m"], V_mps=ref["V"],
                       rho_kgpm3=ref["rho"], qbar_pa=ref["qbar"],
                       alpha_trim_deg=ref["alpha_trim_deg"],
                       mach=ref["V"] / float(atm.a(ref["h_m"])),
                       knudsen=float(atm.knudsen(ref["h_m"], ref["cbar"]))),
        cases=[])
    for case in cases(ref):
        if not case.get("replay"):
            qbar = ref["qbar"] * case["qbar_factor"]
            V = ref["V"]
            h_m = solve_h0(qbar, V, atm)
            case["flight"] = dict(alpha_deg=case["alpha_deg"], qbar_target=qbar,
                                  V=V, h_m=h_m,
                                  qbar_achieved=qbar_of(h_m, V, atm),
                                  mach=V / float(atm.a(h_m)),
                                  knudsen=float(atm.knudsen(h_m, ref["cbar"])),
                                  Md_amp=ref["Md_amp"] * (qbar / ref["qbar_ref0"]))
            design["cases"].append(dict(
                name=case["name"], group=case["group"],
                alpha_ref_deg=case["alpha_deg"],
                qbar_pa=case["flight"]["qbar_achieved"],
                qbar_factor=case["qbar_factor"],
                h_m=h_m, V_mps=V, mach=case["flight"]["mach"],
                knudsen=case["flight"]["knudsen"],
                Md_amp_effective=[float(v) * (case["flight"]["qbar_achieved"]
                                              / ref["qbar_ref0"])
                                  for v in ref["Md_amp"]]))
        else:
            design["cases"].append(dict(
                name=case["name"], group=case["group"],
                maneuver_deg=case["maneuver_deg"], replay=True))
        out[CFG / "scenarios" / ("%s.yaml" % case["name"])] = render_scenario(case, ref)
        for arm in ("no_cmg", "with_cmg"):
            out[CFG / "cases" / ("%s_%s.yaml" % (case["name"], arm))] = render_arm(case, arm)
    return out, design


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="verify the files on disk match the generator (no write)")
    args = ap.parse_args()
    files, design = build()
    drift = []
    for path, text in files.items():
        old = path.read_text(encoding="utf-8") if path.exists() else None
        if old != text:
            drift.append(str(path.relative_to(ROOT)))
            if not args.check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text, encoding="utf-8")
    OUT.mkdir(parents=True, exist_ok=True)
    design_path = OUT / "design.json"
    design_text = json.dumps(design, indent=2, ensure_ascii=False) + "\n"
    if not design_path.exists() or design_path.read_text(encoding="utf-8") != design_text:
        drift.append(str(design_path.relative_to(ROOT)))
        if not args.check:
            design_path.write_text(design_text, encoding="utf-8")
    if args.check:
        if drift:
            print("[check] %d file(s) out of date:" % len(drift))
            for d in drift:
                print("   ", d)
            return 1
        print("[check] %d configuration files are up to date" % len(files))
        return 0
    print("[build] wrote %d configuration files (%d changed)" % (len(files), len(drift)))
    print("[build] reference: Q0 = %.3f Pa at h = %.0f m, V = %.1f m/s, M = %.2f"
          % (design["reference"]["qbar_pa"], design["reference"]["h_m"],
             design["reference"]["V_mps"], design["reference"]["mach"]))
    for c in design["cases"]:
        if c.get("replay"):
            print("   %-26s replay maneuver %+.0f deg" % (c["name"], c["maneuver_deg"]))
        else:
            print("   %-26s alpha=%+5.1f deg  qbar=%8.3f Pa  h=%7.1f m  M=%.2f"
                  % (c["name"], c["alpha_ref_deg"], c["qbar_pa"], c["h_m"], c["mach"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
