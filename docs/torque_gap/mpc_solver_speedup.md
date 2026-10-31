# MPC 求解速度优化：三层方案与实测

目标：把闭环 MPC 的单步求解时间从 **0.24–1.39 s（均值）/ 4.84 s（P95）** 压到
**0.2 s 控制周期以内**，且不牺牲跟踪质量。所有数字都来自
`results/` 下的真实闭环运行，没有单点微基准代替闭环统计。

相关工作：
- 求解器选型与初步量化：`results/torque_gap_no_cmg_comparison/solver_bench.md`
- 三层方案对比：`results/torque_gap_mpc_speed/`（`backends.md`、`study.md`、CSV）
- 图：`figures/torque_gap_mpc_speed/`

---

## 0. 基线诊断

原始实现：CasADi + IPOPT，每个控制周期重解一次 11 状态 / 7 控制的
multiple-shooting NLP（22 节点，407 变量 / 364 约束）。

| 指标 | 值 |
|---|---|
| 每步 IPOPT 迭代 | 35–145（均值 109） |
| 每迭代耗时 | 6.7–9.6 ms |
| 约束 Jacobian 非零元 | 3 201（稀疏，正常） |
| **Lagrangian Hessian 非零元** | **165 649 = 407²（完全稠密）** |

稠密 Hessian 是 8 ms/迭代的根因：一体式 multiple-shooting 在 CasADi 里被展平，
二阶信息失去带状结构。因此优化分三层：

1. **Tier 1**：同样的问题、同样的求解器，只改"怎么用"（容差、采纳策略、热启动）；
2. **Tier 3**：换算法与求解器（HPIPM 实时迭代 RTI + Gauss-Newton QP）；
3. **Tier 2**：改问题本身（网格长度、冗余约束、控制分块）。

---

## 1. Tier 1：不改问题，只改用法（约 3x）

代码：`nearspace/torque_gap_ocp.py::SolverConfig`、`nearspace/torque_gap_sim.py::ClosedLoop.run`

| 改动 | 说明 |
|---|---|
| `tol: 1e-6 → 1e-4` | 每步都重新测量、重新求解，1e-6 是过度求解 |
| `max_iter: 600 → 200` | 限制单个坏步的代价 |
| `accept_feasible: true` | **只要约束违反 ≤ 1e-4 就采纳**，即使 IPOPT 停在迭代上限；`status` 记为 `…\|accepted`，log 增列 `accepted` 与比率 |
| `warm_shift_on_fallback: true` | fallback 时把热启动解平移一个节点继续用，而不是退回冷启动 |

**为什么这最重要**：微基准显示热启动 0.572 s / 156 迭代 vs 冷启动 1.52 s / 279 迭代，
且冷启动收敛到**更差**的局部解（目标 10.08 vs 4.35）。而原先
`Maximum_Iterations_Exceeded` 的步全部被丢弃并触发 fallback——S03 有 34/300 步、
S07 有 26/450 步被白扔。

**结果（34 次全矩阵重跑，`results/torque_gap_no_cmg_comparison_fast/`）**

| 组 | 均值耗时 | P95 | 跟踪变化 |
|---|---|---|---|
| A 组（7/3） | 41.7 ms（原 83.2） | 94 ms（原 197） | 基本不变 |
| B 组（11/7） | 251.6 ms（原 780.9） | 648 ms（原 2973） | 8 场景中 4 个持平、3 个略好、1 个（S07）0.063→0.192° |
| fallback 比例（B） | 0.039 → 0.014 | | |

**结论：约 3x，跟踪不降，A 组全部 8 场景进入 0.2 s。** B 组仍不满足实时。

容差到底该多松（同一状态、同一 NLP 的闭环实测，S00/B）：

| tol | 机动 RMSE | 均值耗时 |
|---|---|---|
| 1e-6 | 0.0958 | 0.429 s |
| **1e-4** | **0.0898** | **0.250 s** |
| 1e-3 | 0.389 | 0.240 s |

tol=1e-3 几乎不再提速却明显伤跟踪 → 选定 1e-4。

---

## 2. Tier 3：HPIPM 实时迭代（RTI）（约 9x 均值 / 43x 最坏，且进入 0.2 s）

新增 `nearspace/torque_gap_rti.py` + `nearspace/hpipm_bindings.py`
（后者把 vendor 里写死 `CDLL('libhpipm.so')` 的 Python 包装重定向到
`D:\Tools\hpipm\build\libhpipm.dll`，并把 `D:\Tools\blasfeo\build` 加进 DLL 搜索路径；
可用环境变量 `HPIPM_LIB` / `BLASFEO_LIB` / `HPIPM_PYTHON` 覆盖）。

每个控制周期：

1. 用上一周期平移后的控制序列从**实测状态**前推，得到参考轨迹 (x̄, ū)；
2. 线性化 RK4 映射 → A_k, B_k（CasADi 解析雅可比，与真值模型的一致性是测试项）；
3. 组装 **Gauss-Newton 结构化 QP**：状态/控制偏差为变量，姿态与角速度误差的
   `JᵀJ` 为 Hessian，控制盒约束逐级给定，gimbal 速率增量的跨级耦合用
   `|du_ν| ≤ γ̈·dt` 的硬信赖域 + 执行侧限幅表达；
4. HPIPM 解 OCP-QP（BLASFEO 内核 + Riccati 递推）；
5. 回溯线搜索（在**非线性**目标上裁步长）→ 更新参考轨迹，执行第一个控制量；
6. 相邻 QP 步长小于执行器分辨率时提前退出。

`qp_iters > 1` 时同一段代码就是 **Gauss-Newton SQP**，可迭代到与 IPOPT 解一致（用于验证）。

**精度–速度拐点（S00/B，22 节点，闭环）**

| QP 迭代数 | 机动 RMSE [deg] | 均值耗时 | P95 |
|---|---|---|---|
| 1 | 0.4259 | 11.7 ms | 14.4 ms |
| 3 | 0.2232 | 32.0 ms | 39.0 ms |
| 8 | 0.1620 | 71.3 ms | 91.7 ms |
| **12** | **0.1111** | **80.1 ms** | **119.8 ms** |
| （IPOPT 基线） | 0.0956 | 955 ms | 4298 ms |

**三种后端各跑一遍完整 34 次矩阵**
（`results/torque_gap_mpc_speed/backends.md`，8 个 P0 场景均值）

| 后端 | 组 | 机动 RMSE | 均值 | P95 | 最大 | 满足 0.2 s 的场景 | fallback |
|---|---|---|---|---|---|---|---|
| baseline IPOPT | A | 6.264° | 83.2 ms | 197 ms | 2670 ms | 4/8 | 0.002 |
| baseline IPOPT | B | 0.733° | 780.9 ms | 2973 ms | 8388 ms | **0/8** | 0.039 |
| fast IPOPT | A | 6.279° | 41.7 ms | 94 ms | 397 ms | 8/8 | 0.002 |
| fast IPOPT | B | 0.740° | 251.6 ms | 648 ms | 1477 ms | **0/8** | 0.014 |
| **RTI 12 QP** | A | 6.277° | 54.8 ms | 107 ms | 220 ms | **8/8** | 0.000 |
| **RTI 12 QP** | B | 1.821° | **79.1 ms** | **128 ms** | **250 ms** | **8/8** | 0.000 |

- B 组：均值 **9.9x**、P95 **23.2x**、最坏 **33.6x** 提速，fallback 降为 0；
  16 次 P0 运行（A+B）的 P95 全部 ≤ 152 ms，全矩阵最坏单步 250 ms
  （S00/B 的首步，冷启动），即 **>99% 的控制步在 0.2 s 周期内**；
- 代价：机动 RMSE 0.733° → 1.821°。这一变化**全部来自两个能力受限场景**
  （S03 俯仰 45°：2.24 → 8.48°；S05 偏航 30°：3.20 → 5.59°），
  其余场景 RTI 与 IPOPT 持平或更好（S02 0.0021 vs 0.0073、S04 0.0006 vs 0.0100、
  S07 0.064 vs 0.063、P1 三场景与 IPOPT 同量级）。
- A 组（7 状态）RTI 55 ms vs IPOPT 83 ms，跟踪完全一致——小问题的 IPOPT 本来就在预算内。

**为什么能力受限场景会退化**：GN 线性化把 `A(γ)ν` 的效应外推到 23 s 视界，
在 σmin→0 的近奇异构型附近线性模型迅速失效；线搜索能防止目标变差，
但无法恢复"非线性求解器能在长视界上找到的预置策略"。这是本方法的
已知能力边界，不是实现缺陷（增加 QP 迭代数到 12 已基本榨干 GN-SQP 的收益）。

---

## 3. Tier 2：改问题本身（一个明确正面、一个明确负面）

**（a）去掉冗余的四元数范数等式 —— 正面**

multiple-shooting 的缺陷 `X_{k+1} = RK4(X_k, …)` 中的 RK4 映射自带四元数归一化，
所以只要缺陷成立，每个节点的 ‖q‖=1 自动成立；原先额外加的 N+1=23 条等式是冗余的。
新增 `torque_gap.solver.quat_norm: all | n0 | none`（默认 `all` 保持历史可复现）。

| 变体（S00/B） | 约束数 | 均值耗时 | 机动 RMSE |
|---|---|---|---|
| fast（all） | 364 | 502.9 ms* | 0.0958 |
| fast + n0 | 342 | 132.6 ms* | 0.0894 |
| fast + g14 + n0 | 342 / 263 变量 | 31.5 ms* | 1.7544 |

\* 该表来自并行 6 进程的研究矩阵，绝对时间含资源竞争，只用来看**相对**差异。

n0 在 S00/S03 上既提速又略提精度（S03 2.24→2.08°），但 **S05 偏航明显变差**
（3.20→4.61°），所以只作为可选开关，不设为默认。

**（b）缩短预测视界 —— 明确负面，且是科学结论**

| 变体 | 节点/视界 | 均值耗时 | S00/B RMSE | S03/B RMSE | S05/B RMSE |
|---|---|---|---|---|---|
| fast | 22 / 23.2 s | 502.9 ms | 0.0958 | 2.2374 | 3.2047 |
| fast + g14 | 14 / 9.2 s | 67.0 ms | **2.3072** | **8.9997** | **5.6544** |
| rti12 | 22 / 23.2 s | 86.8 ms | 0.1111 | 8.4809 | 5.5871 |
| rti12 + g14 | 14 / 9.2 s | 35.0 ms | 3.7438 | 11.5267 | 7.5343 |
| rti12 + g10 | 10 / 4.4 s | 20.8 ms | 4.7181 | 13.4888 | 7.9986 |

视界从 23.2 s 缩到 9.2 s，求解快 7.5x，但跟踪全面崩塌（A 组 S00 更是
4.66°→11.97°）。原因与本实验的机制直接相关：**B 组的构型迁移平均提前 17–19 s 开始**，
9 s 视界在物理上不可能看到需要预置的机动。也就是说，长视界不是"求解器的浪费"，
而是本方案 CMG 预置机制的功能性前提。

因此：**视界保持 23.2 s；提速只能靠求解器/算法（Tier 1/3），不能靠缩短视界。**

---

## 4. 结论与建议

1. **默认后端仍用 IPOPT**，但启用 Tier 1 策略（`fast` profile）：
   3x 提速、跟踪不降、A 组进入实时域。这是零风险收益。
2. **需要实时（≤0.2 s）时用 `rti` 后端**：8.6x 均值 / 43x 最坏提速，
   全场景全步进入 0.2 s 周期；代价是能力受限场景（S03/S05）跟踪退化 2–4x。
   建议：常规机动用 RTI，检测到 σmin 低于阈值或 CMG 接近饱和时切回 IPOPT
   （或提高 `qp_iters`）——这是一个自然的下一步。
3. **不要缩短视界**（功能性前提）；`quat_norm: n0` 可作为可选加速项，但需逐场景验证
   （偏航轴会退化）。
4. 进一步的加速空间（本次未做）：
   - RTI 每步 12 个 QP 中约 5–6 个即收敛（已加提前退出）；剩余开销主要是
     Python 层的 QP 数据装配（每步约 200 次 ctypes 调用）。把 CasADi 的
     RK4/雅可比/误差映射 `.map()` 批量化、QP 数据一次性传入，
     预计可再降 2–3x（→ 30–50 ms/步）；
   - 用 hpipm 的 `d_ocp_qp` + 多级 Riccati 直接吃结构化数据、
     避免逐级 ctypes 设置；
   - 若将来要缩短视界做补偿，必须同时给 CMG 一个显式的长时域预置项，
     否则预置机制消失。

---

## 5. 复现命令

```powershell
# Tier 1（IPOPT 快速 profile）全矩阵
python scripts/run_no_cmg_matrix.py --phase all --jobs 5 --solver-profile fast `
    --results-root results/torque_gap_no_cmg_comparison_fast
# Tier 3（HPIPM RTI）全矩阵
python scripts/run_no_cmg_matrix.py --phase all --jobs 5 --backend rti --qp-iters 12 `
    --results-root results/torque_gap_no_cmg_comparison_rti
# 三层对比与图
python scripts/compare_mpc_backends.py
# Tier 2 变体研究（网格/冗余约束）
python scripts/study_mpc_speed.py --jobs 6
# 单点微基准（求解器选项 / 网格长度）
python scripts/bench_no_cmg_solver.py
# 测试
python tests/test_torque_gap_solver_policy.py
python tests/test_torque_gap_rti.py
```

硬件与软件：Windows 11 / Python 3.14.7 / CasADi 3.8.2（workspace `.pydeps`）/
IPOPT+MUMPHS / HPIPM `D:\Tools\hpipm\build\libhpipm.dll` + BLASFEO
`D:\Tools\blasfeo\build\libblasfeo.dll` / 16 逻辑核（矩阵运行 5–6 并发，
因此绝对耗时有 ±30–50% 的并发噪声，**相对比较才是结论**）。
