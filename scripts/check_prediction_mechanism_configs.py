"""Config audit + grid check for the prediction-mechanism experiment (stage 2).

Loads the three case configs, walks every leaf of the *resolved* configuration and
reports the pairwise differences, so that the only intended experimental variables
(A/B: prediction horizon, A/C: presence of the future manoeuvre) can be verified
before any run is made.  Also prints each group's prediction grid.

  results/torque_gap_prediction_mechanism/config_diff.md
  results/torque_gap_prediction_mechanism/config_diff.json

python scripts/check_prediction_mechanism_configs.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import load_config, dump                    # noqa: E402
from nearspace.torque_gap_scenario import Scenario                # noqa: E402

CASES = {
    "A": "A_full_horizon",
    "B": "B_short_horizon",
    "C": "C_no_future_maneuver",
}
CFG_DIR = ROOT / "configs" / "torque_gap_prediction_mechanism" / "cases"
OUT = ROOT / "results" / "torque_gap_prediction_mechanism"

# leaves that are allowed (indeed required) to differ
ALLOWED = {
    ("A", "B"): {"torque_gap.horizon"},
    ("A", "C"): {"torque_gap.horizon", "trajectory3.maneuver.delta_deg"},
    ("B", "C"): {"torque_gap.horizon", "trajectory3.maneuver.delta_deg"},
}


def flat(obj, prefix=""):
    out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(flat(v, prefix + str(k) + "."))
    elif isinstance(obj, list):
        out[prefix[:-1]] = json.dumps(obj, sort_keys=True)
    else:
        out[prefix[:-1]] = obj
    return out


def main():
    try:                                  # the Windows console is GBK
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                       # noqa: BLE001
        pass
    cfgs, grids = {}, {}
    for tag, name in CASES.items():
        cfg = load_config(str(CFG_DIR / (name + ".yaml")))
        cfgs[tag] = flat(dump(cfg))
        scn = Scenario(cfg)
        grids[tag] = dict(
            n_steps=int(scn.dts.size), horizon_s=float(scn.rel_times[-1]),
            dts=[float(x) for x in scn.dts],
            dt_first=float(scn.dts[0]),
            maneuver_window=[float(m.t0) for m in scn.traj.maneuvers if m.enabled],
            maneuver_delta_deg=[float(abs(m.delta)) * 180.0 / 3.141592653589793
                                for m in scn.traj.maneuvers if m.enabled],
            gamma_limit_mode=scn.gamma_limit_mode,
            gdot_max=float(scn.prm.cmg.gdot_max),
            gddot_max=float(scn.prm.cmg.gddot_max),
            h0=float(scn.prm.cmg.h0),
            cmg_enabled=bool(scn.cmg_enabled),
        )

    pairs = [("A", "B"), ("A", "C"), ("B", "C")]
    rows, complaints = [], []
    for a, b in pairs:
        keys = sorted(set(cfgs[a]) | set(cfgs[b]))
        diffs = [(k, cfgs[a].get(k), cfgs[b].get(k)) for k in keys
                 if cfgs[a].get(k) != cfgs[b].get(k)]
        unexpected = [d for d in diffs if d[0] not in ALLOWED[(a, b)]]
        rows.append(dict(pair="%s vs %s" % (a, b), diffs=diffs,
                         unexpected=unexpected))
        if unexpected:
            complaints.append("%s vs %s: unexpected diffs %s"
                              % (a, b, [d[0] for d in unexpected]))

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "config_diff.json").write_text(json.dumps(
        dict(pairs=rows, grids=grids, complaints=complaints), indent=2,
        ensure_ascii=False), encoding="utf-8")

    md = ["# 配置审计（阶段 2）", "",
          "由 `python scripts/check_prediction_mechanism_configs.py` 生成。"
          "逐叶子比较三组**解析后**的完整配置。", "",
          "## 1. 预测网格与关键约束", "",
          "| 组 | 节点数 | 时域 [s] | 首步 dt [s] | 机动开始 [s] | 机动幅值 [°] | "
          "γ 行程 | γ̇max | γ̈max | h0 |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for tag in ("A", "B", "C"):
        g = grids[tag]
        md.append("| %s | %d | %.1f | %.2f | %s | %s | %s | %.2f | %.1f | %.1f |"
                  % (tag, g["n_steps"], g["horizon_s"], g["dt_first"],
                     ", ".join("%.0f" % v for v in g["maneuver_window"]) or "无",
                     ", ".join("%.1f" % v for v in g["maneuver_delta_deg"]) or "无",
                     g["gamma_limit_mode"], g["gdot_max"], g["gddot_max"], g["h0"]))
    md += ["", "推论：A/B 的机动在 t = 110 s 开始，因此 "
           "**A 在 t = 90 s 时已经能看到机动（23.2 s 时域）**，而 "
           "**B 只能在 t = 105 s 才第一次看到（5.0 s 时域）**。", "",
           "## 2. 两两配置差异", ""]
    for r in rows:
        md.append("### %s" % r["pair"])
        md.append("")
        if not r["diffs"]:
            md.append("- 完全一致。")
        else:
            md.append("| 键 | 前一值 | 后一值 | 是否允许 |")
            md.append("|---|---|---|---|")
            for k, va, vb in r["diffs"]:
                md.append("| `%s` | `%s` | `%s` | %s |"
                          % (k, str(va)[:60], str(vb)[:60],
                             "允许" if k in ALLOWED[tuple(
                                 r["pair"].split(" vs "))] else "**不允许**"))
        md.append("")
    md += ["## 3. 结论", ""]
    if complaints:
        for c in complaints:
            md.append("- **FAIL**: " + c)
    else:
        md.append("- 通过：A/B 只差预测时域；A/C（及 B/C）只差预测时域与未来机动幅值，"
                  "其余（飞行器、CMG、气动盒、扰动、权重、求解器、控制周期）完全一致。")
    (OUT / "config_diff.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md[:40]))
    print("\nwritten:", OUT / "config_diff.md")
    return 0 if not complaints else 1


if __name__ == "__main__":
    sys.exit(main())
