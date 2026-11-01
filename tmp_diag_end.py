import csv, json
import numpy as np

def load(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    d = {}
    for k in rows[0]:
        try:
            d[k] = np.array([float(r[k]) if r[k] not in ("", None) else np.nan for r in rows])
        except ValueError:
            d[k] = np.array([r[k] for r in rows], dtype=object)
    return d

def report(tag, path):
    d = load(path)
    t = d["t_next"]
    nu = np.vstack([d["nu%d" % i] for i in range(1, 5)])
    cmg = np.vstack([d["cmg_x"], d["cmg_y"], d["cmg_z"]])
    hn = d["H_norm"]; sm = d["sigma_min"]; err = d["att_err_deg"]
    aero = np.abs(d["hi_y"] - d["lo_y"])          # aero pitch authority
    # sign flips of the CMG pitch torque per second, windowed
    s = np.sign(cmg[1])
    flips = np.abs(np.diff(s)) > 1
    print("== %s" % tag)
    for lo, hi in ((90, 110), (110, 130), (130, 145), (145, 150.5)):
        m = (t >= lo) & (t < hi)
        if not m.any():
            continue
        print("  %5.0f-%5.0f s: |nu|max %5.2f  nu_sat %.2f  |cmg_y| max %6.1f  "
              "flips/s %5.1f  |H| %6.1f  sigma_min %7.4f  err %6.3f  box_y %6.1f" % (
            lo, hi, np.nanmax(np.abs(nu[:, m])), np.mean(np.abs(nu[:, m]) >= 0.999),
            np.nanmax(np.abs(cmg[1, m])), flips[m[:-1]].sum() / (hi - lo),
            np.nanmax(hn[m]), np.nanmin(sm[m]), np.nanmax(err[m]), np.nanmax(aero[m])))

for tag, path in (
    ("rti2 S00 with_cmg", "results/torque_gap_no_cmg_comparison_rti2/P0/S00_pitch30_with_cmg/timeseries.csv"),
    ("rti2 S05 with_cmg", "results/torque_gap_no_cmg_comparison_rti2/P0/S05_yaw30_with_cmg/timeseries.csv"),
    ("rti2 S03 with_cmg", "results/torque_gap_no_cmg_comparison_rti2/P0/S03_pitch45_with_cmg/timeseries.csv"),
    ("baseline S05 with_cmg", "results/torque_gap_no_cmg_comparison/P0/S05_yaw30_with_cmg/timeseries.csv"),
):
    try:
        report(tag, path)
    except Exception as e:
        print(tag, "FAILED", e)
