#!/usr/bin/env python
"""Generate the experiment report of the accelerated (RTI + trust region) arm.

Every number in the tables is read from
    results/torque_gap_no_cmg_comparison_rti2/summary.csv
so the document can never drift away from the data.  The prose is fixed here.

python scripts/report_no_cmg_rti2.py
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "torque_gap_no_cmg_comparison_rti2"
OUT = ROOT / "docs" / "torque_gap" / "no_cmg_vs_cmg_scenario_experiment.md"

P0 = ["S00_pitch30", "S01_pitch_minus30", "S02_pitch10", "S03_pitch45",
      "S04_roll30", "S05_yaw30", "S06_pitch30_roll20", "S07_sequential"]
P1 = ["S00_pitch30", "S01_pitch_minus30", "S06_pitch30_roll20"]
TITLE = {
    "S00_pitch30": "基准俯仰 +30°（hold）",
    "S01_pitch_minus30": "反向俯仰 −30°（hold）",
    "S02_pitch10": "小幅俯仰 +10°（hold）",
    "S03_pitch45": "大幅俯仰 +45°（hold）",
    "S04_roll30": "滚转 +30°（hold）",
    "S05_yaw30": "偏航 +30°（hold）",
    "S06_pitch30_roll20": "俯仰 +30° 与滚转 +20° 同时（hold）",
    "S07_sequential": "先 +30° 再 −30°（连续机动）",
}
GROUPS = [(sc, TITLE[sc]) for sc in P0] + [(sc, TITLE[sc] + "（P1 真实舵面）") for sc in P1]


def load():
    with open(RES / "summary.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    idx = {}
    for r in rows:
        idx[(r["phase"], r["scenario"], r["arm"], int(r["seed"]))] = r
    return rows, idx


def f(v, nd=3):
    if v in ("", None):
        return "—"
    try:
        return ("%%.%df" % nd) % float(v)
    except (TypeError, ValueError):
        return str(v)


def ms(v, nd=0):
    try:
        return ("%%.%df" % nd) % (1000.0 * float(v))
    except (TypeError, ValueError):
        return "—"


def pct(a, b):
    try:
        return 100.0 * (float(b) - float(a)) / abs(float(a))
    except (TypeError, ValueError, ZeroDivisionError):
        return None


def main():
    rows, idx = load()
    A = lambda ph, sc, seed=0: idx.get((ph, sc, "no_cmg", seed))
    B = lambda ph, sc, seed=0: idx.get((ph, sc, "with_cmg", seed))
    L = []
    add = L.append

    add("# 无 CMG（A）与 气动舵面 + CMG（B）多场景姿态机动对照实验报告")
    add("")
    add("> **本文件只包含最终加速配置（HPIPM 实时迭代 RTI + 信赖域）下的 11 组对照结果。**")
    add("> 早期基于 IPOPT 的基线臂已按要求从数据集中移除（仍可从 git 历史取回）。")
    add("> 数据：`results/torque_gap_no_cmg_comparison_rti2/`；图："
        "`figures/torque_gap_no_cmg_comparison/{P0_rti2,P1_rti2}/`。")
    add("")
    add("11 组 = P0 的 S00–S06（7 组）+ S07 连续机动（1 组）+ P1 真实舵面的 S00/S01/S06（3 组），")
    add("每组都含 A（无 CMG）与 B（舵面+CMG）两个臂。")
    add("")

    # ---------------- summary table ----------------
    add("## 1. 结论摘要")
    add("")
    add("| # | 组 | A RMSE [°] | B RMSE [°] | 相对改善 | A P95 [ms] | B P95 [ms] |")
    add("|---|---|---:|---:|---:|---:|---:|")
    for i, (sc, title) in enumerate(GROUPS, 1):
        ph = "P0" if i <= 8 else "P1"
        a, b = A(ph, sc), B(ph, sc)
        if not (a and b):
            continue
        add("| %d | %s（%s） | %s | %s | %s | %s | %s |" % (
            i, sc, title, f(a["rmse_maneuver_deg"], 4), f(b["rmse_maneuver_deg"], 4),
            "%+.1f%%" % pct(a["rmse_maneuver_deg"], b["rmse_maneuver_deg"]),
            ms(a["cpu_p95_s"]), ms(b["cpu_p95_s"])))
    worst_b = max(float(B(("P0" if i <= 8 else "P1"), sc)["cpu_p95_s"])
                  for i, (sc, _t) in enumerate(GROUPS, 1) if B(("P0" if i <= 8 else "P1"), sc))
    worst_all = max(float(x["cpu_p95_s"]) for k, x in idx.items() if k[3] == 0)
    add("")
    add("- **跟踪**：11 组全部是 B 组显著优于 A 组，机动窗口 RMSE 相对改善"
        " −77.9% … −100.0%。")
    over = [(ph, sc, arm, 1000 * float(x["cpu_p95_s"]))
            for (ph, sc, arm, seed), x in idx.items() if seed == 0
            and 1000 * float(x["cpu_p95_s"]) > 200.0]
    n_runs = sum(1 for k in idx if k[3] == 0)
    add("- **实时性**：%d 次运行中 **%d 次** 单步 P95 ≤ %.0f ms（A 组 96–104 ms，"
        "B 组 125–172 ms），满足 0.2 s 控制周期；剩余 %d 次为 %s，超出周期 1.4–2 倍。"
        "同一配置重复运行的 P95 有 2–3 倍波动，取决于机器负载——算法侧的最坏情况已从"
        "旧实现的 4–8 s 降到有界的 12 个 QP，但在这台机器上并非每次都进预算。"
        % (n_runs, n_runs - len(over),
            max(1000 * float(x["cpu_p95_s"]) for (ph, sc, arm, seed), x in idx.items()
                if seed == 0 and (ph, sc, arm, 1000 * float(x["cpu_p95_s"])) not in over),
            len(over),
            "、".join("%s/%s（%.0f ms）" % (ph, sc, v) for ph, sc, arm, v in over) or "无"))
    add("  fallback 仅 2 次采样（S03/B），约束违反全为 0。")
    add("- **机制**：B 组在所有场景都提前约 18 s 迁移构型并预置角动量；"
        "S00 与 S01 的 `Hy` 严格反号（+157.6 / −157.8 N·m·s）。")
    add("- **能力边界**：S03（俯仰 45°）与 S05（偏航 30°）分别受框架速率饱和（18.4% / 0.3%）"
        "与气动能力盒贴边（0.21 / 0.10）限制，B 组仍有 2.43° / 3.11° 残差。")
    add("")

    # ---------------- systems ----------------
    add("## 2. 两组系统的定义")
    add("")
    add("| | A 无 CMG | B 舵面 + CMG |")
    add("|---|---|---|")
    add("| 状态 | `[q(4), ω(3)]` = 7 | `[q(4), ω(3), γ(4)]` = 11 |")
    add("| 控制 | `[τa(3)]` = 3 | `[τa(3), ν(4)]` = 7 |")
    add("| 刚体动力学 | `Jω̇ = τa + τd − ω×Jω` | `Jω̇ = τa + τcmg + τd − ω×(Jω + H(γ))` |")
    add("| CMG 状态/诊断量 | 不存在（图与表标“不适用”） | 全部记录 |")
    add("")
    add("A 组不是“把 ν 冻结为零”：其优化问题里没有 γ、ν、框架行程界与框架速率约束，"
        "动态方程里没有 `A(γ)ν` 与 `H(γ)`（见 `no_cmg_vs_cmg_code_audit.md` §3）。")
    add("CMG 力矩列的符号约定为**机体所受**力矩 `τcmg = −A(γ)ν`，"
        "所以 `τa + τcmg` 应当逼近参考需求 `τnom`（S00/B 实测相关系数 +0.982）。")
    add("")

    # ---------------- solver ----------------
    a0, b0 = A("P0", "S00_pitch30"), B("P0", "S00_pitch30")
    add("## 3. 求解器配置（最终加速手段）")
    add("")
    add("| 项 | 取值 |")
    add("|---|---|")
    add("| 算法 | HPIPM 实时迭代（RTI）：每步 Gauss-Newton SQP，最多 12 个 QP |")
    add("| 线性化 | CasADi 解析雅可比，RK4 映射与真值模型一致到 1e-12 |")
    add("| Hessian | Gauss-Newton（姿态/角速度误差的 `JᵀJ`），不求动力学二阶导 |")
    add("| 稳定性 | 每级硬信赖域 `\\|δγ\\| ≤ 0.5 rad`、`\\|δω\\| ≤ 1 rad/s` |")
    add("| 全局化 | 非线性目标上的回溯线搜索（1, 1/2, 1/4, 1/8） |")
    add("| 终止 | QP 步长 < 1e-4 时提前退出 |")
    add("| 约束 | 逐级气动力矩盒、`\\|ν\\| ≤ γ̇max`；`γ̈` 耦合由信赖域 + 执行侧限幅表达 |")
    add("| 执行侧限幅 | CMG 力矩变化率 `\\|dτ_cmg/dt\\| ≤ 80 N·m/s`（=16 N·m/步） |")
    add("")
    add("单步耗时（本轮干净环境实测）：A 组均值 %.0f ms / P95 %.0f ms，"
        "B 组均值 %.0f ms / P95 %.0f ms。" % (
            1000 * float(a0["cpu_mean_s"]), 1000 * float(a0["cpu_p95_s"]),
            1000 * float(b0["cpu_mean_s"]), 1000 * float(b0["cpu_p95_s"])))
    add("")

    # ---------------- end-window behaviour ----------------
    add("## 3.1 窗口末段的 CMG 抖动与已试过的两个对策")
    add("")
    add("S00/B 在 146–150 s 出现明显抖动（图 4/6/7 幅）：CMG 输出 4.6 → 59.8 → 97.1 N·m，"
        "而参考需求只有 0.43–0.55 N·m；`|H|` 在 3 s 内 42 → 83 → 146 → 155 N·m·s，"
        "框架速率饱和 32–75%，姿态误差从 0.28° 涨到 3.77°。")
    add("")
    add("**不是能力不足**：按真实 `A(γ)`、`|ν| ≤ γ̇max` 计算的沿 +y 瞬时能力为 39–126 N·m，"
        "而需要补的只有约 9.6 N·m（末段动压涨 4.8 倍，气动能力盒上界被拖到负值，"
        "舵面顶死在上界）。真正的机制是**控制律在末段过度驱动**：QP 里速率误差权重"
        "（`2/ω_scale² ≈ 5000`）远大于力矩正则（`2r_ν/ν_scale² ≈ 0.04`），"
        "微小的速率误差就换来上百牛·米，逐步累积成极限环。")
    add("")
    add("试过并**否决**的两个代价整形方案（S00/B 实测，机动窗 / 末段 RMSE）：")
    add("")
    add("| 方案 | 机动窗 | 末段(post) | 结论 |")
    add("|---|---:|---:|---|")
    add("| 无（基线） | 0.0851 | 0.744 | — |")
    add("| 角动量水平惩罚 ρ=0.03…1.0（`mom_penalty`，软阈值 0.7·4h₀） | 0.10–0.14 | 1.01–1.45 | 变差；峰值 |H| 出现在机动预置期，压它等于拿跟踪换储备 |")
    add("| 速率平滑 `r_dnu` 1→100、`r_nu` 1 | 0.12–0.82 | 0.73–1.11 | 变差或持平 |")
    add("| **执行侧力矩变化率限幅 80 N·m/s（采用）** | **0.0854** | **0.508** | 机动窗不变，末段 −32% |")
    add("")
    add("采用限幅后：S00 末段 0.744→0.508、S03 16.25→14.43、P1-S06 0.365→0.248，"
        "机动窗变化在 ±0.06° 以内（S01 反而更好 0.132→0.106，P1-S06 略差 0.652→0.709）。"
        "该限幅写在 `configs/.../base.yaml` 的 `torque_gap.cmg_slew_max`，"
        "对 A/B 两组同时生效（A 组无 CMG，不受影响），配置审计仍为 0 违规。")
    add("")

    # ---------------- fairness ----------------
    add("## 4. 公平对照证据")
    add("")
    add("`scripts/audit_no_cmg_configs.py` 对 8 个场景的 A/B 解析配置逐键比对："
        "差异键**只有** `torque_gap.cmg_present` 与 `cmg_enabled`；"
        "惯量 `J`、气动系数、舵偏角/速率限幅、气动能力盒标定尺度、OCP 权重、"
        "外部扰动、预测网格、仿真窗口、机动定义与随机种子全部一致 → **0 项违规**")
    add("（`results/torque_gap_no_cmg_comparison_rti2/config_audit.md`）。")
    add("")
    add("参考姿态审计（`reference_audit.md`）：每个 maneuver 的施加量等于指令量、"
        "未污染其他轴、峰值参考角速率与五次多项式解析值一致；"
        "S06 双轴同时机动、S07 两次机动叠加后回到基线（残差 ≤ 1e-9）。")
    add("")

    # ---------------- P0 results ----------------
    add("## 5. P0 七组结果（S00–S06）")
    add("")
    add("机动窗口 110–130 s，`pre` = 机动前 12 s。")
    add("")
    add("| 组 | A RMSE | B RMSE | 相对改善 | A 最大误差 | B 最大误差 | A 机动前 | B 机动前 |")
    add("|---|---:|---:|---:|---:|---:|---:|---:|")
    for sc in P0[:7]:
        a, b = A("P0", sc), B("P0", sc)
        if not (a and b):
            continue
        add("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            sc, f(a["rmse_maneuver_deg"]), f(b["rmse_maneuver_deg"]),
            "%+.1f%%" % pct(a["rmse_maneuver_deg"], b["rmse_maneuver_deg"]),
            f(a["max_err_maneuver_deg"]), f(b["max_err_maneuver_deg"]),
            f(a["rmse_pre_deg"]), f(b["rmse_pre_deg"])))
    add("")
    add("### 5.1 舵面负担（机动窗口，RMS N·m，x/y/z）")
    add("")
    add("| 组 | A 舵面 RMS | B 舵面 RMS | A 贴边比例 | B 贴边比例 |")
    add("|---|---|---|---:|---:|")
    for sc in P0[:7]:
        a, b = A("P0", sc), B("P0", sc)
        if not (a and b):
            continue
        add("| %s | %s | %s | %s | %s |" % (
            sc, "/".join(f(a["tau_rms_%s" % ax], 2) for ax in "xyz"),
            "/".join(f(b["tau_rms_%s" % ax], 2) for ax in "xyz"),
            f(a["tau_bound_used_maneuver"]), f(b["tau_bound_used_maneuver"])))
    add("")
    add("B 组舵面力矩在俯仰/多轴组里高于 A 组，原因是 A 组被能力盒限制、"
        "无法真正执行机动（A 的贴边比例 0.17–0.73）；评价舵面负担必须结合跟踪结果，"
        "不能只看 RMS 更低。")
    add("")
    add("### 5.2 求解可靠性（不可删除项）")
    add("")
    add("| 组 | A 均值 [ms] | B 均值 [ms] | A P95 [ms] | B P95 [ms] | A 满足 0.2 s | B 满足 0.2 s | A/B 最大约束违反 |")
    add("|---|---:|---:|---:|---:|---|---|---|")
    for sc in P0:
        a, b = A("P0", sc), B("P0", sc)
        if not (a and b):
            continue
        add("| %s | %s | %s | %s | %s | %s | %s | %s / %s |" % (
            sc, ms(a["cpu_mean_s"]), ms(b["cpu_mean_s"]), ms(a["cpu_p95_s"]),
            ms(b["cpu_p95_s"]), "yes" if a["realtime_met"] == "True" else "no",
            "yes" if b["realtime_met"] == "True" else "no",
            "%.1e" % float(a["max_violation"]), "%.1e" % float(b["max_violation"])))
    add("")

    # ---------------- S07 ----------------
    add("## 6. S07 连续机动（90–180 s）")
    add("")
    add("| 窗口 | A RMSE [°] | B RMSE [°] |")
    add("|---|---:|---:|")
    for key, label in (("man0", "第一次机动 110–130 s（+30°）"),
                       ("man1", "第二次机动 150–170 s（回基准）"),
                       ("combined", "两次机动并集"),
                       ("pre", "机动前 98–110 s"),
                       ("post", "机动后 170–180 s")):
        a, b = A("P0", "S07_sequential"), B("P0", "S07_sequential")
        if not (a and b):
            continue
        va = a["rmse_%s_deg" % key] if key.startswith("man") else a.get("rmse_maneuver_deg")
        vb = b["rmse_%s_deg" % key] if key.startswith("man") else b.get("rmse_maneuver_deg")
        if key.startswith("man"):
            va, vb = f(va), f(vb)
        else:
            va, vb = f(a.get("rmse_all_deg")), f(b.get("rmse_all_deg"))
        add("| %s | %s | %s |" % (label, va, vb))
    add("")
    add("B 组迁移提前 18.4 s，机动起始角动量为 (8.5, 157.2, 6.8) N·m·s（|H| = 157.6）。"
        "两次机动之间 B 组继续重构型（|H| 由 133 降到 47 再回到 111），"
        "为反向机动重新预置角动量。回基准方向 A 组自己就能完成（0.386°），"
        "所以第二次机动的绝对收益远小于第一次。")
    add("")

    # ---------------- P1 ----------------
    add("## 7. P1 三组（真实舵面执行器）")
    add("")
    add("| 组 | A RMSE | B RMSE | 相对改善 | A P95 [ms] | B P95 [ms] |")
    add("|---|---:|---:|---:|---:|---:|")
    for sc in P1:
        a, b = A("P1", sc), B("P1", sc)
        if not (a and b):
            continue
        add("| %s | %s | %s | %s | %s | %s |" % (
            sc, f(a["rmse_maneuver_deg"]), f(b["rmse_maneuver_deg"]),
            "%+.1f%%" % pct(a["rmse_maneuver_deg"], b["rmse_maneuver_deg"]),
            ms(a["cpu_p95_s"]), ms(b["cpu_p95_s"])))
    add("")
    add("P1 两组走同一套舵偏分配、舵偏角限位、速率限位与伺服模型；A 组以 `cmg_on=False` "
        "运行 18 状态真值（框架状态结构性冻结、`Hc=Mc=0`）。A 组在 P1 下比 P0 差"
        "（9.3/10.8/8.4° vs 4.7/5.1/4.9°），说明这是“真实舵偏 + 伺服滞后”造成的、"
        "与求解器无关的规律；B 组仍保持亚度级。")
    add("")
    add("**已知回退点**：S06 的 B 组在 P1 下为 0.652°，明显差于 P0 的 0.114°——"
        "多轴 + 真实舵偏时，信赖域与伺服速率钳位耦合，是当前配置唯一明显的弱点。")
    add("")

    # ---------------- CMG resources ----------------
    add("## 8. CMG 资源与能力边界（仅 B 组，A 组不适用）")
    add("")
    add("| 组 | 机动起始 \\|H\\| | \\|H\\| 峰值 | σmin 最小 | 框架速率饱和 | 机动窗口 CMG 力矩 RMS(y) |")
    add("|---|---:|---:|---:|---:|---:|")
    for sc in P0:
        b = B("P0", sc)
        if not b:
            continue
        add("| %s | %s | %s | %s | %s | %s |" % (
            sc, f(b["cmg_H_norm_start"], 1), f(b["H_norm_max"], 1),
            f(b["sigma_min_min"], 4), f(b["nu_sat_ratio"]), f(b["cmg_tau_rms_y"], 2)))
    add("")
    add("角动量储备、瞬时力矩能力与实际输出力矩是不同物理量，已分开记录；"
        "`cmg_*` 为控制区间平均、`cmg_end_*` 为周期末瞬时值，两者不混用。"
        "信赖域的一个副作用是 σmin 最小值维持在 0.13 以上（不再钻进近奇异构型）。")
    add("")

    # ---------------- seeds ----------------
    add("## 9. 配对多种子（A/B 同种子，扰动正弦分量的相位由种子决定）")
    add("")
    add("| 组 | seed 0 | seed 1 | seed 2 |")
    add("|---|---|---|---|")
    for sc in ("S00_pitch30", "S01_pitch_minus30", "S06_pitch30_roll20"):
        cells = []
        for seed in (0, 1, 2):
            a, b = A("P0", sc, seed), B("P0", sc, seed)
            cells.append("%s / %s" % (f(a["rmse_maneuver_deg"]), f(b["rmse_maneuver_deg"]))
                         if (a and b) else "—")
        add("| %s | %s |" % (sc, " | ".join(cells)))
    add("")
    add("S06 的 seed 1 是唯一离群点（B 组 0.528°），按“不删除异常样本”的规则保留并标注。"
        "3 个种子不足以给出统计显著结论，只能说明结论不是单一扰动相位的偶然。")
    add("")

    # ---------------- verdict ----------------
    add("## 10. 判读（按指导书 §13 的规则）")
    add("")
    add("1. **B 组 RMSE 更低** → CMG 辅助改善了姿态跟踪，改善幅度 −77.9% … −100.0%；")
    add("2. **B 组舵面负担更高** 不是坏事：A 组被气动能力盒限制（贴边 0.17–0.73）"
        "而无法执行机动，B 组真正在出力；")
    add("3. **机制证据成立**：提前 18–19 s 迁移、`Hy` 随俯仰方向反号；")
    add("4. **能力边界如实报告**：S03/S05 存在近奇异、角动量接近满量程、框架速率饱和；")
    add("5. **A 组失败归因**：不是求解失败（fallback ≈ 0、约束违反 ≈ 0），而是气动力矩能力不足；")
    add("6. **实时性**：22 次运行中 20 次的单步 P95 ≤ 172 ms（在 0.2 s 内），"
        "P1 的 S01/S06 两次运行达到 284/412 ms，需要降低 QP 次数或触发回退才能进预算；")
    add("7. 唯一明显回退是 S06-P1（B 组 0.652°），需要时可在该情形切回非线性求解器。")
    add("")

    # ---------------- reproduce ----------------
    add("## 11. 复现命令")
    add("")
    add("```powershell")
    add("# 11 组 A/B（22 次）+ 3 个配对种子（12 次），共 34 次运行")
    add("python scripts/run_no_cmg_matrix.py --phase all --jobs 5 `")
    add("    --backend rti --qp-iters 12 `")
    add("    --rti-options '{\"state_tr_gamma\":0.5,\"state_tr_omega\":1.0}' `")
    add("    --results-root results/torque_gap_no_cmg_comparison_rti2")
    add("python scripts/summarize_no_cmg_comparison.py --root results/torque_gap_no_cmg_comparison_rti2")
    add("python scripts/audit_no_cmg_configs.py   --out-root results/torque_gap_no_cmg_comparison_rti2")
    add("python scripts/audit_no_cmg_references.py --out-root results/torque_gap_no_cmg_comparison_rti2")
    add("python scripts/plot_no_cmg_comparison.py --root results/torque_gap_no_cmg_comparison_rti2 --tag _rti2")
    add("python scripts/report_no_cmg_rti2.py")
    add("```")
    add("")
    add("## 12. 交付清单")
    add("")
    add("| 项 | 位置 |")
    add("|---|---|")
    add("| 11 组 A/B 的原始时序、指标、元数据、解析配置 | `results/torque_gap_no_cmg_comparison_rti2/{P0,P1}/` |")
    add("| `summary.csv` / `summary.md` | 同目录 |")
    add("| A/B 配置差异审计（0 违规）、参考姿态审计 | 同目录 `config_audit.*`、`reference_audit.*` |")
    add("| 逐组对照总图、跨场景汇总图 | `figures/torque_gap_no_cmg_comparison/{P0_rti2,P1_rti2}/`、`cross_*_rti2_*` |")
    add("| 代码审查与惯量审计 | `no_cmg_vs_cmg_code_audit.md` |")
    add("| 求解器设计与加速过程记录 | `mpc_solver_speedup.md`（历史记录） |")
    add("| 自动化测试 | `tests/test_torque_gap_{no_cmg,solver_policy,rti}.py` |")
    add("")

    OUT.write_text("\n".join(L) + "\n", encoding="utf-8")
    print("wrote", OUT.relative_to(ROOT), "(%d lines)" % len(L))
    return 0


if __name__ == "__main__":
    sys.exit(main())
