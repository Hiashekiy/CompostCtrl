# 力矩级 MPC：八顶点力矩盒认证与舵机二分反求的解析加速

> 依据：`力矩级MPC_八顶点认证与舵机二分加速_详细实施方案.md`
> 基准提交：`33f36f7`（力矩级 MPC、4 轮 SQP、0.2 s、含 DT01/DT02 参数化）
> 实验分支：`experiment/fast-torque-authority`
> 冻结条件：`dt_ctrl=0.2 s`、**最多 4 轮 SQP**（绝不 5 轮）、真实舵机 `1 ms`、原 6DOF/气动/CMG/参考/权重/分配器全部未改、无恒力矩扰动。
> 生产默认仍是 `legacy`；`analytic_full` 只是实验分支的可选开关。

## 1. 问题与现状

在线预测每一阶段都要（1）构造三轴内接力矩盒并逐个校验八个顶点，（2）对八个顶点的舵偏目标各做 45 轮二分反求一条恒定舵机命令。历史分项计时（一次 22 阶段预测）：`box ≈ 2.10 ms`、`certificate ≈ 8.58 ms`，合计 ~10.7 ms/整条预测，仅次于平动预测 8.21 ms。

原实现证明的是**给定阶段末端舵偏、对应末端力矩目标的可达性**；它没有、本实验也没有声称保持期间输出恒定目标力矩。

## 2. 数学依据（等价，不放松）

1. **力矩盒**：`τ = τ_nat + Bδ`，`B = q̄·M`，`M` 固定。以对称舵偏半径 `r` 与中心 `c` 构造盒 `{τ_c+e : |e| ≤ h}`。则**盒内全部点（含八顶点）落在对称舵偏区内的充要条件**是逐元素
   `|B⁻¹|·h ≤ r`。
   一次不等式取代八次分配器调用。`c、r、h₀=|B|r`、连通轴分组、组内缩放 `s` 与 `(1−1e-12)` 余量**沿用原函数**，因此盒的上下限与轴缩放完全不变；顶点舵偏目标用一次矩阵乘 `c + B⁻¹(h∘signs)` 生成，顶点力矩残差用同式向量化计算（不是填 0）。
2. **舵机证书**：真实 1 ms 舵机在常值命令下端点映射 `F_n(u)` 对 `u` 单调非降（一阶惯性 + 对称速率限幅 + `h_s ≤ τ_s`）。于是合法命令区间可达的端点集合恰为区间 `[F_n(lo), F_n(hi)]`，任何落在其中的顶点目标都**存在**一条常值可达命令 —— 这就是原 45 轮二分要证的东西，只是不需要求那条命令。
   因此正常路径**不求逆命令、不写 0 残差**：新增 `commands_evaluated=false`、`certified`、`endpoint_margin`，`max_residual_rad=NaN`、`command=[]`。生产首控仍来自 `bounded_control_allocation` 链路，与该证书无关。

**凡是解析条件不成立的一律回退 legacy**：`rcond(B) ≤ 1e-10`、盒不可证、顶点残差超容差、端点区间不满足、参数前提不满足。回退计数独立记录（`analytic_fallback_count`、`certificate_type`），不以“更多回退”冒充加速。

## 3. 改动清单

| 文件 | 改动 |
|---|---|
| `simulink/hpipm/online_fast_torque_box.m`（新） | 解析力矩盒：沿用原 `c/r/h₀/分组/缩放/余量`，用 `|B⁻¹|h ≤ r` 一次判定并向量化生成顶点目标与残差；`certificate_type='analytic_box'`；异常回退 legacy 并标注 `legacy_singular` / `legacy_unprovable` |
| `simulink/hpipm/online_fast_servo_certificate.m`（新） | 单调区间证书：闭式算 `F_n(lo)、F_n(hi)`，检查全部顶点落在区间内并给出最小边界余量；`commands_evaluated=false`、`max_residual_rad=NaN`；不可证则 `certified=false` 由调用方回退 |
| `simulink/hpipm/online_authority_rollout.m` | 新增 `s.authority_certificate_mode`（`legacy` / `analytic_box` / `analytic_full` / `shadow`）；只替换证书两处调用；新增逐阶段分项计时 `phase_time_s`（air/aero/reachability/box/servo_certificate/allocation/attitude/translation/total）与证书字段（`certificate_type`、`certified`、`endpoint_margin`、`box_margin`、`commands_evaluated`、`analytic_fallback_count`） |
| `simulink/hpipm/verify_authority_candidate.m` | 解析模式下按阶段判定：解析阶段看 `certified` 与 `endpoint_margin`，legacy 回退阶段看其自身数值残差；不再用“残差 0”冒充未执行的检查；CMG/分配残差/首控检查原样保留 |
| `simulink/modelgen/build_closedloop.m` | 新增第 9 个可选参数 `certificate_mode`（默认 `legacy`），仅实验使用 |

未改动：`native_online_rti_step.m`（4 轮 SQP、接受机制、信任域）、QP 与权重、`online_translation_prediction`、`online_surface_reachability`、`bounded_control_allocation`、`native_surface_allocation_sfun`、真值模型、参考与扰动。

## 4. 等价性验收（`check_fast_authority_equivalence`，硬断言）

```
FAST_AUTHORITY_EQUIVALENCE_PASS boxes=1500 servo=400 real=1144 box_err=0.00e+00 margin=0.00e+00
```

1. **1500 个随机合法工况**（含 `q̄` 1e3–1e5、随机 `M` 扰动、退化半径 `r→0`、不对称区间）：解析盒与 legacy 的 `tau_min/tau_max` 最大绝对误差 **0.0 N·m**；`axis_scale`、8 个顶点舵偏目标同样 **0.0**（阈值 1e-12）。legacy 自身拒绝的工况不计入（两侧都必须可构造）。
2. **400 个舵机场景**（`δ₀` 全行程、4 种阶段长度、正/反向、速率饱和、接近限幅）：闭式端点与独立 1 ms 递推逐点一致（<1e-12）；落在可达区间内的目标全部通过解析证书（假阴性单独计数），区间外的目标 **0 次**被误判为可达；同一目标交给 legacy 45 轮二分亦收敛到 <1e-9 rad。
3. **1144 个真实闭环阶段**（来自已保存的 60 s 运行逐阶段重放）：解析盒与 legacy 误差 **0.0 N·m**，端点余量全部非负。

## 5. 微基准（同机、同一组工况）

| 阶段 | legacy | 解析 | 加速 |
|---|---|---|---|
| 力矩盒（每次调用） | 0.069 ms | 0.037 ms | 1.9× |
| 舵机证书（每次调用） | 0.377 ms | 0.036 ms | 10.5× |
| 完整 22 阶段 prediction | 22.0 ms | 13.0 ms | **1.69×** |

完整 prediction 的输出（`native_F_*` 传播后的 `X`）在四种模式下**逐元素相同**（断言 `<1e-12`）。

## 6. 60 s 闭环对照（4 轮 / 0.2 s / 301 周期，0 次回退）

| 模式 | 臂 | 姿态 RMS/° | 峰值/° | 周期均值 | P50 | P95 | P99 | 最大 | >200 ms | box ms | 舵机证书 ms | 解析占比 | 回退 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| legacy | A | 3.825 | 8.324 | 72.2 | — | 125.7 | — | 548.8 | 2 | 1.86 | 7.85 | 0 | 0 |
| analytic_full | A | **3.825** | 8.324 | **50.5** | — | **101.9** | — | **199.3** | **0** | 1.35 | **0.96** | 100% | 0 |
| legacy | B | 1.1098 | 3.616 | 81.7 | — | 135.6 | — | 220.4 | 1 | 1.89 | 7.92 | 0 | 0 |
| analytic_full | B | **1.1098** | 3.616 | **59.6** | — | **112.5** | — | 276.5 | **3** | 1.32 | **0.93** | 100% | 0 |
| analytic_box | B | 1.1098 | 3.616 | 88.7 | — | 148.9 | — | 392.8 | 5 | 1.42 | 8.70 | 100%(盒) | 0 |

要点：

- **控制结果与 legacy 完全一致**（A 3.825°/B 1.1098°，两臂均与 legacy 逐位相同），说明加速是数值等价的，不是靠改控制换来的。
- A 臂：均值 72.2 → 50.5 ms（−30%），P95 125.7 → 101.9 ms，**超 200 ms 周期 2 → 0**，最大 548.8 → 199.3 ms。
- B 臂：均值 81.7 → 59.6 ms（−27%），P95 135.6 → 112.5 ms，但该次运行超 200 ms 周期 1 → 3。为此另跑了一对重复（legacy 与 analytic_full 各再 60 s）：

  | B 臂重复 | 均值 | P95 | 最大 | >200 ms |
  |---|---|---|---|---|
  | legacy（第 2 次） | 84.0 ms | 133.3 ms | 306.4 ms | 4 |
  | analytic_full（第 2 次） | 54.4 ms | 86.4 ms | 272.4 ms | 4 |

  即**均值与 P95 稳定改善约 30%，超 200 ms 次数落在 1–4 之间且 legacy 自身也有 1–4 的波动**（最大 220–306 ms）。样本只有各 2 次，机器负载噪声与长尾机制尚未分离，因此**不能声称 B 臂预算次数一定不增加**，也不反过来声称它一定变差。
- `analytic_box`（只换盒、保留二分）在 B 臂均值反而略差（88.7 ms），说明收益主要来自单调区间证书，而不是盒。
- 舵机证书分项由 7.85/7.92 ms 降到 0.96/0.93 ms（≈8×），与微基准的 10× 一致；盒分项 1.86/1.89 → 1.35/1.32 ms。

## 7. 结论与处置（按方案 §7）

1. 静态等价、边界/奇异回退、真实阶段重放**全部通过**，没有出现 fast 独有的放行；解析证书没有扩大任何区间。
2. 控制输出与 legacy 一致，无新增物理安全回退（两模式均 0 回退）。
3. 认证时间显著下降（舵机证书 ≈8×、整条 prediction 1.69×）；A 臂均值/P95/超预算次数全部改善；B 臂均值与 P95 稳定改善约 30%，超 200 ms 次数在两次重复中为 1、4（legacy）对 3、4（analytic），**样本不足以下"不增加"的结论**。
4. 依据方案停止条件（第 3 条无法用现有样本证明）：**`analytic_full` 不设为默认**，生产默认保持 `legacy`；开关可秒级切换（`build_closedloop(...,'legacy')`）。这是保守处置，不是否定加速本身——等价性与分项耗时证据是充分的。
5. 未做：方案 S3 的可选消融（自适应信任域、命令级速率约束）与平动预测优化 —— 后者仍是最大单项（≈8 ms/rollout），本轮按方案只动证书。

## 8. 复现命令

```powershell
cd D:\项目\飞轮
# 等价性与微基准
.\simulink\mlab.cmd "addpath('simulink'); init_paths; check_fast_authority_equivalence; benchmark_fast_authority(400)"
# 2 s 冒烟 + 60 s 对照（legacy / analytic_box / analytic_full / shadow）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; run_fast_authority_suite('smoke'); run_fast_authority_suite('main60')"
# 汇总
.\simulink\mlab.cmd "addpath('simulink'); init_paths; report_fast_authority"
```

## 9. 结果文件

- 等价性：`simulink/results/online_authority/fast_certificate/fast_authority_equivalence.{json,mat}`
- 微基准：`.../fast_certificate/fast_authority_benchmark.{json,mat}`
- 闭环对照：`.../fast_certificate/ablation/fast_authority_comparison.{csv,json}`，每个配置目录内 `summary.json`、`run_config.json`、`<臂>_closedloop.mat`
- 逐阶段计时与证书类型：`<配置>/…_authority.mat` 中的 `authority.phase_time_s`、`authority.certificate_type`、`authority.endpoint_margin`

## 10. 尚不能证明之处

1. B 臂的长尾（max 276.5 ms、3 个超预算周期）未定位到具体机制；需要用 profile 判断是 MATLAB GC/调度还是某类工况的解析回退（本运行解析占比 100%，回退 0）。
2. 端到端 1.69× 与单次调用 10× 的差异说明证书已不是瓶颈；剩余主项是平动预测与 MATLAB 调度，本轮按方案未动。
3. 所有耗时为本机 Windows 同步仿真墙钟，不等于硬件实时；legacy 本身也有超 200 ms 周期。
