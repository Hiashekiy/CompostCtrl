import json, subprocess, sys
jobs = [("A_180", "no_cmg", 180.0), ("B_180", "with_cmg", 180.0)]
procs = []
for name, arm, end in jobs:
    spec = dict(run_id=name, phase="ext", scenario="S00_pitch30", arm=arm,
                case="configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_%s.yaml" % arm,
                plant="P0", seed=0, seed_randomize=False, out_dir="results/_ext/%s" % name,
                solver_profile="baseline", backend="rti", qp_iters=12,
                overrides={"torque_gap": {"end": end,
                                          "rti": {"state_tr_gamma": 0.5, "state_tr_omega": 1.0}}})
    procs.append((name, arm, subprocess.Popen(
        [sys.executable, "scripts/run_no_cmg_matrix.py", "--run-one", json.dumps(spec)],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)))
for name, arm, p in procs:
    rc = p.wait()
    try:
        m = json.load(open("results/_ext/%s/metrics.json" % name, encoding="utf-8"))
        st = m["stats"]; a = st["all"]
        print("%-6s rc=%d steps %3d  man %7.4f  post %8.4f  all %8.4f  Hmax %6s  nusat %6s  cpu %5.3f" % (
            name, rc, m["n_steps"], st["combined"]["rmse_deg"],
            (st.get("post") or {}).get("rmse_deg", float("nan")), a["rmse_deg"],
            ("%.1f" % a["H_norm_max"]) if "H_norm_max" in a else "-",
            ("%.3f" % a["nu_sat_ratio"]) if "nu_sat_ratio" in a else "-",
            m["cpu_s"]["mean"]), flush=True)
    except Exception as e:
        print(name, "rc=", rc, "FAILED", e, flush=True)
