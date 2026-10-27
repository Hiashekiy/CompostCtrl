# legacy —— 归档代码（不在当前实验链路内）

本目录保存**被 CMG A/B 研究取代**的早期实现，只为了可追溯性保留，不参与
`scripts/` 中任何流程，也不再维护。当前有效代码只有 `nearspace/` + `scripts/`。

## 内容与归档原因

| 文件 | 内容 | 为什么归档 |
|---|---|---|
| `model.py` | 俯仰通道 5 状态统一模型（θ, q, δ, τ, h_w） | 只覆盖俯仰轴；审稿意见明确要求三轴，已被 `nearspace/model6c.py`（18 状态 + CMG）取代 |
| `mpc.py` | 俯仰通道 LTV-MPC + `LTVMPC` | `QPSolver` 已抽出到 `nearspace/qp.py`，其余部分随俯仰通道模型一起归档 |
| `mpc3.py` | 反作用轮三轴 LTV-MPC（16 状态） | 反作用轮方案因工程不可行（60 N·m 需 31 kW）被 CMG 取代 |
| `controllers.py`, `controllers3.py` | "参考力矩 + 链式分配"基线控制器 | 对照基线，已由 `nearspace/cmg_control.py` 的统一 MPC 承担 |
| `simulate.py`, `simulate3.py` | 俯仰 / 三轴闭环仿真与指标 | 已由 `nearspace/sim.py` + `nearspace/metrics.py` 取代 |
| `mpc_cmg.py` | 第一代 CMG MPC（直接优化框架速率 γ̇） | 因奇异附近预测失配发散被 `cmg_control.LTVMPCCMG2` 取代，见 `docs/CMG-实现状态.md` §4–§5 |
| `cmg_hier.py` | 分层 CMG 控制器（动量级 MPC + 内环操纵律） | 分层方案没有通过闭环验证，且 A/B 对比要求"同一控制器只换执行机构"，故不作为实验方案 |
| `run_all.py`, `run_three_axis.py` | RW 时期 C0–C4 / T1–T3 实验入口 | 结果数据在 `results/legacy_rw/` |
| `run_cmg.py`, `run_cmg_compare.py` | CMG 时期的第一/第二轮对比入口 | 已被 `scripts/run_cmg_ab.py` 取代；其数据在 `results/cmg_ab/baseline_legacy/` |

## 使用方式

这些模块的相对 import 已改写为绝对 import，可以在工作区根目录直接 import：

```python
import legacy.model, legacy.mpc, legacy.mpc3, legacy.cmg_hier   # 均可导入
```

但 `legacy/run_*.py` 里的 `HERE` 仍指向本目录，运行会把输出写到 `legacy/results/`。

## 归档过程中丢失的内容（如实记录）

* 早期 RW / CMG 图件生成脚本（`make_figures.py`、`make_execution_figures.py`、
  `make_actuator_figures.py`、`make_cmg_compare_figure.py`、`make_cmg_role_figure.py`）
  在归档移动时被错误地按 ANSI 编码读写，中文字符串损坏，已删除；它们生成的 PNG
  本身仍在 `results/legacy_rw/` 与 `results/cmg_ab/baseline_legacy/`。
  这些图若需要重新生成，须按当前 `scripts/plot_cmg_ab.py` 的风格重写。
* `results/cmp_A.csv`（145 s、仅舵面、100 km 顶点脉冲工况）在整理前的一次试运行中
  被覆盖，已删除。该工况由当前流程的 `disturbance` A 工况完整复现
  （同高度、同脉冲、同初值），对应数据见 `results/cmg_ab/disturbance/A/`。
  `results/cmg_ab/baseline_legacy/cmp_B.csv` 仍是旧代码的原始输出。
