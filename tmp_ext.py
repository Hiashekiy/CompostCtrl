import json, subprocess, sys
jobs = [("A_150", "no_cmg", 150.0), ("A_180", "no_cmg", 180.0),
        ("B_150", "with_cmg", 150.0), ("B_180", "with_cmg", 180.0)]
procs = []
for name, arm, end in jobs:
    spec = dict(run_id=name, phase="ext", scenario="S00_pitch30", arm=arm,
                case="configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_%s.yaml" % arm,
                plant="P0", seed=0, seed_randomize=False,
                out_dir="results/_ext/%s" % name, solver_profile="baseline",
                backend="rti", qp_iters=12,
                overrides={"torque_gap": {"end": end,
                                          "rti": {"state_tr_gamma": 0.5, "state_tr_omega": 1.0}}})
    procs.append((name, subprocess.Popen(
        [sys.executable, "scripts/run_no_cmg_matrix.py", "--run-one", json.dumps(spec)],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)))
for name, p in procs:
    p.wait()
    m = json.load(open("results/_ext/%s/metrics.json" % name, encoding="utf-8"))
    st = m["stats"]
    print("%-6s steps %3d  man %7.4f  post %8.4f  all %8.4f  |H|max %6.1f  nusat %5.3f  cpu %5.3f" % (
        name, m["n_steps"], st["combined"]["rmse_deg"], (st.get("post") or {}).get("rmse_deg", float("nan")),
        st["all"]["rmse_deg"], st["all"]["H_norm_max"], st["all"]["nu_sat_ratio"], m["cpu_s"]["mean"]), flush=True)
