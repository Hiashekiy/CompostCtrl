import csv, json
import numpy as np
def load(p):
    rows = list(csv.DictReader(open(p, encoding="utf-8")))
    out = {}
    for k in rows[0]:
        if k == "status":
            continue
        try:
            out[k] = np.array([float(r[k]) if r[k] not in ("", None) else np.nan for r in rows])
        except ValueError:
            pass
    return out
b150 = load("results/_ext/B_150_dbg/timeseries.csv")
b180 = load("results/_ext/B_180/timeseries.csv")
a150 = load("results/_ext/A_150/timeseries.csv")
a180 = load("results/_ext/A_180/timeseries.csv")

m = lambda d: (d["t_next"] >= 140.0) & (d["t_next"] <= 149.9)
print("overlap 140-150 s, B_150 vs B_180 (max abs difference):")
for k in ("att_err_deg", "tau_act_y", "cmg_y", "H_norm", "nu1"):
    print("   %-12s %.3e" % (k, float(np.nanmax(np.abs(b150[k][m(b150)] - b180[k][m(b180)])))) )
print("overlap 140-150 s, A_150 vs A_180:")
for k in ("att_err_deg", "tau_act_y", "wy"):
    print("   %-12s %.3e" % (k, float(np.nanmax(np.abs(a150[k][m(a150)] - a180[k][m(a180)])))))
print()
print("authority box calibration scale: 150-end %.6f   180-end %.6f" % (
    json.load(open("results/_ext/B_150_dbg/meta.json", encoding="utf-8"))["experiment"]["authority_scale"],
    json.load(open("results/_ext/B_180/meta.json", encoding="utf-8"))["experiment"]["authority_scale"]))
print()
print("%6s %8s %8s %8s %8s %8s %8s %8s" % ("t", "errA", "errB", "hi_y", "tau_nom", "cmg_y", "|H|", "Q"))
for tt in (140, 145, 150, 155, 160, 165, 170, 175, 180):
    i = int(np.argmin(np.abs(b180["t_next"] - tt)))
    j = int(np.argmin(np.abs(a180["t_next"] - tt)))
    print("%6.0f %8.2f %8.2f %8.2f %8.2f %8.2f %8.1f %8.2f" % (
        b180["t_next"][i], a180["att_err_deg"][j], b180["att_err_deg"][i],
        b180["hi_y"][i], b180["tau_nom_y"][i], b180["cmg_y"][i], b180["H_norm"][i], b180["Q"][i]))
print()
for tag, d in (("A(no CMG)", a180), ("B(+CMG)", b180)):
    t, e = d["t_next"], d["att_err_deg"]
    bits = []
    for thr in (5, 10, 20, 40):
        idx = np.flatnonzero((e > thr) & (t > 130.0))
        bits.append(">%d deg: %s" % (thr, ("%.1f s" % t[idx[0]]) if idx.size else "never"))
    print("%-10s %s" % (tag, ", ".join(bits)))
