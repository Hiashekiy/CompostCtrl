import csv
import numpy as np
def load(p):
    rows=list(csv.DictReader(open(p,encoding="utf-8"))); d={}
    for k in rows[0]:
        try: d[k]=np.array([float(r[k]) if r[k] not in ("",None) else np.nan for r in rows])
        except ValueError: d[k]=np.array([r[k] for r in rows],dtype=object)
    return d
d=load("results/torque_gap_no_cmg_comparison_rti2/P0/S00_pitch30_with_cmg/timeseries.csv")
t=d["t_next"]
wr=np.vstack([d["wrefx"],d["wrefy"],d["wrefz"]])
print(" %6s %8s %9s %9s %9s %9s %9s %9s %9s %s" % ("t","|w_ref|","d|wref|/dt","tau_nom_y","d_y","lo_y","hi_y","h_km","Q","err"))
for tt in (120,130,138,142,144,146,147,148,149,150):
    i=int(np.argmin(np.abs(t-tt)))
    dref = np.linalg.norm(wr[:,i]-wr[:,max(i-1,0)])/max(t[i]-t[max(i-1,0)],1e-9)
    print(" %6.1f %8.4f %9.4f %9.2f %9.2f %9.2f %9.2f %9.3f %9.5f %.3f" % (
        t[i], np.linalg.norm(wr[:,i]), dref, d["tau_nom_y"][i], d["d_y"][i],
        d["lo_y"][i], d["hi_y"][i], d["h_km"][i], d["Q"][i], d["att_err_deg"][i]))
print()
print("disturbance (scaled by q_inf) and reference-rate growth, both arms:")
for arm in ("no_cmg","with_cmg"):
    for root,tag in (("torque_gap_no_cmg_comparison_rti2","rti2"),("torque_gap_no_cmg_comparison","ipopt")):
        p="results/%s/P0/S00_pitch30_%s/timeseries.csv"%(root,arm)
        dd=load(p); tt=dd["t_next"]
        w=np.vstack([dd["wrefx"],dd["wrefy"],dd["wrefz"]])
        def at(ts,arr):
            i=int(np.argmin(np.abs(tt-ts))); return arr[i]
        rate=lambda ts: np.linalg.norm(at(ts+0.2,w)-at(ts,w))/0.2
        print("  %-9s %-5s |wref| 130=%6.4f 150=%6.4f ; d|wref|/dt 130=%7.5f 150=%7.5f ; err 130=%.3f 150=%.3f" % (
            tag, arm, at(130,w).__abs__().sum(), np.linalg.norm(at(150,w)),
            rate(130), rate(150), at(130,dd["att_err_deg"]), at(150,dd["att_err_deg"])))
