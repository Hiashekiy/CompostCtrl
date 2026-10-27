# 近空间稀薄大气飞行器气动舵面 / CMG 协同姿态控制

在**同一套**飞行器真值模型、大气模型、参考姿态、初始状态和统一 MPC 框架下，
只切换执行机构集合，对比

* **方案 A —— `surface_only`**：仅气动舵面；
* **方案 B —— `surface_cmg`**：气动舵面 + 4 单元单框架 CMG 金字塔（联合优化）。

研究问题、验收标准和统计口径见 `docs/CMG-协同姿态控制实验实施方案.md`；
本次实施的逐项结果见 `docs/CMG-协同姿态控制实验-实施报告.md`；
权重与动量约束的专项核验见 `docs/CMG-权重与动量约束核验.md`。

> 结论性提醒：本仓库不预设"CMG 一定更好"。所有 A/B 差异都必须在同一参考、
> 同一扰动、同一初值、同一评分容差下取得，并且要同时报告求解失败、
> 约束违反和角动量积累。

---

## 1. 目录

```
D:\项目\飞轮\
├─ config.yaml                     全部基础参数（飞行器/大气/弹道/控制器权重）
├─ configs\cmg_ab\matrix.yaml      A/B 实验矩阵：common + cases + modes
├─ nearspace\                      有效代码（无遗留）
│  ├─ atmosphere.py                8 段指数大气（legacy 后端，仅用于复现旧数据）
│  ├─ atmosphere_real.py           完整 US76 + NRLMSIS-2.0（pymsis，带 CSV 缓存）
│  ├─ atmosphere_factory.py        后端选择 exp / us76 / msis2
│  ├─ trajectory.py                skip 弹道（h, V, γ）
│  ├─ trajectory3.py               三轴姿态参考 + 五次多项式指令机动 + 扰动
│  ├─ model6.py                    三轴刚体、气动系数、四元数工具
│  ├─ cmg.py                       CMG 金字塔几何、角动量、Jacobian、奇异度量、能力核算
│  ├─ model6c.py                   18 状态真值模型 [q, ω, δ, γ, γ̇]（`cmg_on` 决定 A/B）
│  ├─ qp.py                        行缩放 OSQP + ADMM 回退
│  ├─ cmg_control.py               统一 LTV-MPC（决策量 = [δ_c, M_cmg]）+ 操纵律
│  ├─ metrics.py                   §4.5/§5.5 全部指标与固定评分常数
│  ├─ scenarios.py                 场景解析、权重/预算/CMG 能力核算、机动可行性
│  ├─ sim.py                       闭环仿真、时序记录、元数据
│  └─ config.py                    YAML 叠加加载（base: / extends:）
├─ scripts\
│  ├─ run_cmg_ab.py                统一 A/B 运行入口（配置驱动，唯一变量是执行机构）
│  ├─ verify_cmg_weights.py        §7.1/§7.2 权重链路 + 力矩链路核验
│  ├─ check_maneuver_feasibility.py §5.2 机动可行性检查
│  ├─ evaluate_cmg_ab.py           指标表、改善率、§10.4 判据
│  ├─ plot_cmg_ab.py               §9 总体对比图 + 执行机构机理图
│  └─ run_all_experiments.py       端到端复现（核验 → 仿真 → 评估 → 作图）
├─ tests\
│  ├─ test_atmosphere_real.py      大气后端验证（US76 公认值比对等）
│  └─ test_cmg_ab.py               A/B 机制验证（CMG 能力、A 模式无 CMG、公平性、指标定义）
├─ results\
│  ├─ cmg_ab\<case>\<A|B>\         每次运行的 timeseries.csv / metrics.json / meta.json / config_resolved.yaml
│  ├─ cmg_ab\summary.{csv,md}      A/B 对比与改善率（脚本生成）
│  ├─ cmg_ab\maneuver_feasibility.csv
│  ├─ cmg_ab\weight_verification.json
│  ├─ cmg_ab\baseline_legacy\      旧代码的 CMG 对比原始数据（只读参考）
│  ├─ legacy_rw\                   早期反作用轮实验原始数据（只读参考）
│  └─ _cache\                      NRLMSIS-2.0 查表缓存
├─ figures\cmg_ab\                 全部由 plot_cmg_ab.py 生成
├─ docs\                           实施方案、实施报告、专项核验、建模底稿、汇报材料
└─ legacy\                         已归档的早期实现（不在实验链路内，见 legacy/README.md）
```

## 2. 环境

```
pip install -r requirements.txt        # numpy scipy osqp pyyaml matplotlib pymsis
```

已在 Python 3.14 / numpy 2.5 / scipy 1.18 / osqp 1.1 下验证。

## 3. 运行

```bash
# 端到端复现（核验 → 可行性 → 全部 A/B 工况 → 指标 → 图）
python scripts/run_all_experiments.py --jobs 4

# 单独跑某个工况的 A、B
python scripts/run_cmg_ab.py --case tracking --modes A B
python scripts/run_cmg_ab.py --case maneuver10 maneuver20 --jobs 4

# 只重算指标 / 只重画图（原始 CSV 不动）
python scripts/evaluate_cmg_ab.py
python scripts/plot_cmg_ab.py

# 专项核验
python scripts/verify_cmg_weights.py
python scripts/check_maneuver_feasibility.py

# 测试
python tests/test_atmosphere_real.py
python tests/test_cmg_ab.py
```

`python scripts/run_cmg_ab.py --list` 列出全部工况。

## 4. 公平性：A 与 B 到底差了什么

`configs/cmg_ab/matrix.yaml` 里，`common` 定义环境/弹道/参考/控制器权重，
`cases` 定义任务，`modes` 定义执行机构。A 与 B 的解析后配置**只在三处不同**
（`tests/test_cmg_ab.py::test_fairness` 会断言这一点）：

| 键 | A | B |
|---|---|---|
| `actuators.mode` | `surface_only` | `surface_cmg` |
| `mpc_cmg.M_cmg_box` | 0 N·m（优化器无 CMG 权限） | 61 N·m（保证力矩） |
| 显示标签 | — | — |

而且"没有 CMG"是在**真值模型**里实现的，不只是控制器层：
`VehicleCMGParams.cmg_on=False` 时框架角冻结在零位、框架速率恒为零、
CMG 力矩与 CMG 角动量都恒为零（home 位形本身净角动量为零），
因此 A 是一架真的没有 CMG 的飞行器，而不是"有 CMG 但不用"。

其余一切——高度剖面、速度、NRSMSIS-2.0 大气、气动系数、初始配平状态
（含初始升降舵偏角）、参考姿态、外部扰动、`dt_ctrl=0.2 s`、
`dt_plant=0.005 s`、积分器、指标定义和评分容差——完全相同。

## 5. 关键实现决策

1. **CMG 取代反作用轮**：反作用轮 `M_stall=60 N·m` 在 5000 rpm 需要约 31 kW、
   转子约 22 kg/轴，工程不可行（审稿意见 2.2）。改为 4×50 N·m·s 单框架
   金字塔：全向保证力矩 **86.8 N·m**、单轴可用角动量 **107.7 N·m·s**、
   有利方向峰值 245 N·m、120 kg、0.8 kW。这些数字由
   `scenarios.cmg_spec()` 从几何**算出来**，不是抄来的常数。
2. **统一 MPC 直接输出 CMG 力矩**，不设分配层：决策量 `[δ_c(3), M_cmg(3)]`，
   预测模型中内嵌"阻尼伪逆操纵律 + 零运动 + 一阶速率环"，
   并把可交付力矩包线 `|UᵀM_cmg| ≤ torque_margin·σ_i·√n·γ̇max`
   与 `σ_min(A)` 下限作为硬约束/松弛约束，使奇异附近指令自动收缩。
3. **CMG 是动量交换装置而不是动量汇**：`|H|` 用径向包络
   `H·M_cmg ≥ (|H|²−H_lim²)/(2Δt)` 约束，超预算时它自动变成"禁止继续充电"，
   同时保留垂直于 H 的全部控制权限。逐分量硬盒在越界后必然永久不可行，已弃用。
4. **薄气段禁止卸载**（`q_inf < 40 Pa` 且未超预算时不允许 `H·M_cmg > 0`），
   因为 100 km 以上把动量倒给气动舵面只会把误差倒回机体。
5. **求解成功的判据是约束违反量，不是 OSQP 的状态字符串**：状态
   `solved inaccurate` 但违反量 1e-8 的解是可以用的指令；反之
   3e11 的违反量必须回退。这消掉了旧日志里的"假失败"。
6. **QP 必须关掉 OSQP 内部缩放**：行由我们自己归一化（动量行的元素量级 ~50，
   框架速率行 ~1），开启内部缩放会让 RMSE 从 4.0° 恶化到 58.9°。
7. **增益/权重的生效值必须打印**：配置有 `config.yaml → matrix.yaml → case →
   mode` 四层叠加，只看某一层 YAML 会得出错误结论（详见 §7 核验文档）。

## 6. 结果

全部数字由 `scripts/evaluate_cmg_ab.py` 从原始 `metrics.json` 自动生成，
见 `results/cmg_ab/summary.md`；图见 `figures/cmg_ab/`。
结论与机理分析见 `docs/CMG-协同姿态控制实验-实施报告.md`。

## 7. 已知限制

* 三轴气动系数仍是**线性示例值**（`Cl_beta, Cn_beta, Cl_da, …`），
  没有真实气动表，不能解读为真实飞行器的已验证气动权限；
* 弹道是解析余弦跳跃剖面，不是真实跳跃弹道；
* 主方案**不含 RCS**，因此 89 km 以上的长期动量卸载问题没有解决，
  本仓库也不声称解决；
* MPC 单步求解时间不满足实时：闭环平均 A 299–444 ms、B 514–818 ms
  （`dt_ctrl = 0.2 s`），最坏节点（持续 1° 姿态误差）单进程实测 B 为 1211 ms；
  主要开销是有限差分 Jacobian，尚未做实时性优化（见实施报告 §11）；
* 目标函数**没有 CMG 力矩的速率惩罚项**，B 的舵面与 CMG 力矩存在高频抖振，
  舵面平均动作量是 A 的 1.8–2.3 倍；
* 未做 HIL 验证。
