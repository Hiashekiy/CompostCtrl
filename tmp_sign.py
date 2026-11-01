import csv
import numpy as np
p = "results/torque_gap_no_cmg_comparison_rti2/P0/S00_pitch30_with_cmg/timeseries.csv"
rows = list(csv.DictReader(open(p, encoding="utf-8")))
d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k not in ("status",)}
t = d["t_next"]; m = (t >= 112) & (t <= 128)          # inside the manoeuvre
nom = d["tau_nom_y"][m]; ta = d["tau_act_y"][m]; tc = d["cmg_y"][m]
print("torque balance inside the manoeuvre (S00_B, rti2):")
print("  RMS(tau_nom)            = %7.2f N·m" % np.sqrt(np.mean(nom**2)))
for name, combo in (("tau_a + cmg_logged", ta + tc), ("tau_a - cmg_logged", ta - tc)):
    r = np.corrcoef(combo, nom)[0, 1]
    res = np.sqrt(np.mean((combo - nom) ** 2))
    print("  %-22s RMS=%7.2f  corr(.,tau_nom)=%+.3f  RMS(residual)=%7.2f" % (
        name, np.sqrt(np.mean(combo**2)), r, res))
