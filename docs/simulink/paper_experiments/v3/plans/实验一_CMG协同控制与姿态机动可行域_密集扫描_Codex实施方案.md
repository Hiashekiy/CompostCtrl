# 实验一重设计：CMG 协同控制与姿态机动可行域密集扫描

> **交付给 Codex 的一次性实施任务**。只修改、重测论文**实验一（E1）**；实验二、三、四不重跑、不改结果。**全部仿真使用 MATLAB R2024a + Simulink；禁止使用 Python 进行建模、运行、参考生成或统计。**
>
> 仓库：`Hiashekiy/CompostCtrl`；工作基线为 `codex/paper-experiments` 分支（审查时 HEAD `f60ef63`）。若执行时 HEAD 变化，先检查差异并记录实际 commit；不要假定代码接口永远不变。
>
> 两个正式模型：`simulink/models/NearSpace_CMG_FlightControl.slx`、`simulink/models/NearSpace_CMG_FlightControl_SurfaceOnly.slx`。**只读、不得覆盖、不得重建到这两个路径。**

## 0. 任务目标和必须遵守的边界

本轮只回答：**在相同飞行环境、相同 MPC 及执行器设置下，加入 CMG 能否扩大可完成姿态机动的范围，以及降低达到相同姿态变化的参考过渡时间？**

- 以**俯仰参考增量**为扫描变量，不把“俯仰角变化 25°”误写为“迎角 25°”；真实迎角由速度方向与机体姿态共同决定。
- 不改变六自由度机体、气动力/气动力矩、舵机、CMG 驱动和物理限幅，不禁用 `Assert_alpha_domain` 等保护；不调整 MPC 权重以迎合结果；保持 4 轮 SQP。
- **完整保留** `simulink/results/paper_experiments/20261105_paper_v2/` 中已有 99 份仿真输出及相关报告，不覆盖、删除或重命名原始任务。
- 不“挑选成功工况”后悄悄扔掉失败工况。**所有计划工况均必须留下状态与可追溯数据**；对于气动模型越界的工况，标记“模型适用域外/未判定”，不得当成“CMG 无法完成机动”。
- 不因单独一组仅舵面或 CMG 先失败而停止另一组：A、B 分别运行，最后按相同条件成对比较。
- 实验完全在 **Simulink 六自由度闭环**中进行，不切换 Python，也不退回姿态简化模型。

## 1. 现有代码与基线（先核实，不要凭印象重写）

当前分支已有如下实现，优先复用：

| 文件 | 当前真实用途 | 本轮操作 |
|---|---|---|
| `simulink/paper/run_paper_suite.m` | 原有 4 组实验统一入口；旧 E1 扫描 `[10 20 30 40] × [2 5 10 20]` | **尽量不改原入口行为**；新增专用 E1 密集扫描入口 |
| `simulink/paper/make_paper_reference_bank.m` | 从存档 bank 构造四元数、角速度和预测节点参考；`step` 使用五次平滑多项式 | 复用、必要时增加防呆检查，不改原始物理定义 |
| `simulink/paper/build_paper_case.m` | 调用 `build_closedloop`，生成独立 case 模型，固定在线 MPC/F20 设置 | 复用；确保模型名、场景 ID 和来源正确 |
| `simulink/modelgen/build_closedloop.m` | **第 12 个参数 `bank_path`**允许显式加载每例参考；`case_name` 非空时写到 `simulink/cases/` | 使用此接口，**绝不覆盖正式 `.slx`** |
| `simulink/paper/evaluate_paper_runs.m` | 计算四元数跟踪误差、物理限幅、失败状态等 | 尽量复用指标定义，补强单例有效域诊断与重算能力 |
| `simulink/paper/paper_extra_metrics.m` | 计算 CMG 框架角、动量、力矩、控制器补充指标 | 复用；新批次务必真实执行，不只生成基础 `metrics.json` |
| `simulink/paper/paper_final_report.m` | 汇总原四组实验 | 不要用新批次覆盖旧的四组总报告 |
| `simulink/paper/paper_preflight.m` / `paper_hash.m` | 环境检查和文件哈希 | 用于保护正式模型和默认 bank |
| `simulink/data/paper_banks/archive/S00_pitch30_B_bank.mat` | 既有参考初态与扰动来源 | 用作**同一个** A/B 参考源，生成各自的输入 bank |
| `simulink/data/online_plan_bank.mat` | 默认构建输入，可能与正式 `.slx` 烘焙参考不同 | 不覆盖、不就地替换 |

已有 S00 典型对照参考结果：仅舵面 **3.8703°**、带 CMG **0.3241°**，是原 `E1_baseline` 的 60 s 全程 RMSE；原扫描 16 对中 A 成功 2、B 成功 7，但 30°/40° 频繁触发模型适用域或控制器问题。**新实验的指标定义、参考任务和判据必须写清，不能把不同任务的数值拼成一张直接比较表。**

先用 `git status`、当前分支、commit 和上述 MATLAB 源码核实调用接口，出具新的运行配置快照。若发现分支不符，先报告并明确选择来源分支；**不得擅自重置或丢弃用户的工作区更改**。

## 2. 新实验矩阵：主网格 144 次 A/B 仿真

**俯仰参考增量（9 档，单位 °）**：

```matlab
pitch_steps_deg = [5 7.5 10 12.5 15 17.5 20 22.5 25];
```

**规定参考过渡时间（8 档，单位 s）**：

```matlab
transition_s = [2 4 6 8 10 12 16 20];
```

对每种组合均独立运行：

- **A：仅舵面飞行器**，`NearSpace_CMG_FlightControl_SurfaceOnly` 生成的 case；
- **B：舵面+四台 CMG 飞行器**，`NearSpace_CMG_FlightControl` 生成的 case。

合计 `9 × 8 × 2 = 144` 次（72 对）。**不超过 25° 的主网格**；对于 27.5°、30°，本轮只可在主网格全面验收后、根据有效域结果另做少量**标记为 exploratory** 的边界补充，不纳入 144 次主结果的分母，也不得自动扩展成第二个大规模扫描。

每一例统一：

```matlab
profile      = 'step';                 % 现有五次平滑参考，非瞬时阶跃
onset        = 8;                      % 仿真第 8 s 开始机动
amplitude    = pitch_steps_deg(i);     % 相对初始姿态的俯仰增量
transition   = transition_s(j);       % 从 onset 到 onset+transition
simulation_duration = 60;             % 各组统一、完整 60 s
controller   = 'MPC';
authority_mode = 'online_predicted';
certificate  = 'analytic_full';
torque_shrink_ratio = 0;
inner_mode   = 'feedback_predictive';  % F20：20 ms 下层力矩反馈
inner_period = 0.02;
control_period = 0.2;
sqp_iterations = 4;
physical_step = 0.001;
prediction_dts = production_online_qp.dts; % 原 23.2 s 节点，不擅自改网格
```

**同一个 A/B 配对必须满足**：机动起点、参考四元数序列、参考体角速度序列、初始机体姿态/角速度、初始飞行状态、扰动时间序列、仿真时长、MPC 配置及 F20 下层参数一致；仅植物中是否存在 CMG（以及由此必然不同的状态/控制维数）不同。A 不应继承旧正式生产模型的 `legacy` 证书/旧 0.2 s 分配器。`build_paper_case` 目前将两臂统一配置为 `analytic_full + feedback_predictive`，请**在生成的 `.slx` 中重新核实**，而不是依赖脚本注释。

> 注意：`make_paper_reference_bank` 的初始 `q0` 并不意味着初始俯仰角为 0°，`amplitude` 是**相对于存档初始姿态的增量**。报告需另外记录绝对俯仰角、真实迎角及其峰值。

## 3. 统一判定规则：先分清控制失败、模型越界和跟踪失败

本轮沿用 `criteria_v2.md` 对 `step` 的判据，不因结果差而改规则：

1. 仿真完整到 `60 s`，无非有限状态、无物理限幅断言失败；
2. 令参考过渡结束时刻 `t_f = 8 + transition`；检查 **`[t_f+2, t_f+10]`** 中全部已采样姿态误差，要求四元数最短旋转误差 **始终 ≤1°**；
3. 同时按现有评估器规则检查控制器回退等异常。满足全部条件才标记 `SUCCESS`。

不得把“仿真结束时误差很小”替代上述稳定判据。不能拿中断前的 RMSE 与完整 60 s RMSE 作公平对比。

**状态建议固定为互斥类型**（同时保留细分 `reason` 与原始错误堆栈）：

| 状态 | 判断 | 用途 |
|---|---|---|
| `SUCCESS` | 完整 60 s、在模型有效域内、满足统一跟踪及物理判据 | 可行网格点 |
| `TRACKING_FAIL` | 完整 60 s、状态有效，但未满足姿态误差门槛 | 未通过网格点 |
| `MODEL_DOMAIN_FAIL` | 真值气动力模型或在线预测触发已知物理/表格适用域断言（迎角、气动表或高度范围等），包括提前停止 | **未知/不可评价**，非“机动不可实现” |
| `CONTROLLER_FAIL` | 控制器求解/认证或异常回退超限且不能判为模型越界 | 控制器失效，单独记录 |
| `INFRA_FAIL` | MATLAB 文件、工具链、模型装载、路径或执行错误，没有可信结果 | 必须修复后重跑同例，不计物理性能 |

**重要：** `CONTROLLER_FAIL` 不代表“物理不可达”，只能说明当前控制方案未能完成；不同状态在热力图上必须视觉区分。A/B 对照同时提供每一臂的状态，以及**共同有效域掩码**：`A` 与 `B` 都没有 `MODEL_DOMAIN_FAIL` 和 `INFRA_FAIL`，且都具备完整可靠的可比较数据。若其中一组超域，则该配对不参与“谁更优”的共同有效域成功率统计；仍需在单臂全网格热力图中标注。

### 针对迎角越界的强制诊断

当前 `build_public_force_model.m` 采用升阻系数查表，明确创建了 `Assert_alpha_domain`；`online_flight_condition.m` 同时会检查 `force_polar.mat` 的迎角数据范围和大气高度表范围。**±40° 不是已验证的真实飞行器失速/安全阈值**，只是当前混合参数替代模型的工程保护边界。

必须记录：

- 真实状态计算得到的迎角时间序列 `alpha_deg(t)`、最大绝对迎角、越界前最后时刻；必要时记录高度、速度、动压；
- 报错来源是**物理真值**中的 `Assert_alpha_domain`，还是 **MPC 预测/认证**中的 `online:alphaRange` 或 `online:atmosphereRange`；这两类不要合并为“真实机体已经越界”；
- 中断时 `t_end`、原始 `ErrorMessage` 与堆栈（`CaptureErrors`）、最后有效状态；
- 在中断之前仍输出**明确标为 partial 的轨迹**，绝不输出看起来像完成 60 s 的结果。

优先复用模型已有 `flight_conditions` / `states` / 飞行状态日志。若日志不包含可靠的真值迎角，给实验**case 副本**补一个只读 `ToWorkspace` 真值迎角日志，采用现有 `online_flight_condition` 相同的 `atan2(v_body_z,v_body_x)` 口径；**不要因诊断需求修改物理模块或禁止断言**。同时区分姿态欧拉角与迎角，单位清楚。

## 4. 单例数据采集、存储和可复算性

每个工况必须保留一套独立目录，建议：

```
simulink/results/paper_experiments/<new_stamp>/E1_dense/
  matrix.json
  preflight.json
  criteria.md
  cases/
    E1D_0050_0020_A/
      case_config.json
      build_config.json
      reference_bank.mat
      sim_output.mat             # 若仿真可获得有效原始数据
      run_status.json            # 包含失败源、错误堆栈、耗时
      metrics.json
      extra_metrics.json
      timeseries.csv
      domain_diagnostics.csv
      tracking.png / tracking.pdf
      ...
    E1D_0050_0020_B/
      ...
  tables/
    E1_dense_all_cases.csv
    E1_dense_paired.csv
    E1_dense_min_duration.csv
    E1_dense_failure_details.csv
  figures/
    E1_dense_feasibility_A.png/.pdf
    E1_dense_feasibility_B.png/.pdf
    E1_dense_min_duration.png/.pdf
    E1_dense_typical_tracking.png/.pdf
  E1_dense_report.md
  acceptance.json
```

目录和文件名仅为建议，Codex 可适度调整，但**必须保持稳定的参数映射**（别把 `7.5°` 错编码成 `75°`）以及 JSON/CSV 的双向索引。每对工况有固定 `pair_id`，A/B 使用 `pair_id+A/B`；同一命令重跑不覆盖成功结果，失败可带 revision 或显式 retry 子目录。

**每例必存基础字段**：`case_id`、`pair_id`、`arm`、`pitch_step_deg`、`transition_s`、`onset_s`、`simulation_stop_s`、`actual_end_s`、`status`、`reason`、`source_git_sha`、`bank_sha256`、`model_sha256`、`controller_config`、`reference_profile`、`reference_q0`、`max_abs_alpha_deg`、`alpha_at_end_deg`、`rmse_full_deg`（只有完整运行才作横向比较）、`rmse_partial_deg`（中断时）、`peak_attitude_error_deg`、`post_transition_max_error_deg`、`settled_criterion_pass`、`fallback_count`、`wall_time_s`、`error_message`。

可利用现有日志增加解释性字段：舵面各轴偏角/偏转速率饱和占比、目标/实际气动力矩 RMSE、CMG 框架角及角速度峰值、CMG 力矩峰值或 RMS、角动量范数、控制器每周期计算耗时均值/P95/最大值和超过 200 ms 的比例。**请区分实际测得量、日志推导量和理论速率盒上界**；CMG 力矩若由框架角数值微分重建，注明差分误差，勿当作直接传感器真值。

上述指标应当来自**Simulink 原始输出**与 `vehicle_params`，禁止通过“看图估数字”或编造缺失字段生成漂亮的论文表。

## 5. 实现范围：只新增 E1 专用批次，不破坏现有四组实验

新增专用 MATLAB 文件，例如：

1. `simulink/paper/run_E1_dense.m`：构建完整 144 例矩阵，提供 `preflight`、`smoke`、`run`、`analyze`、`report`（可另加 `retry_failed`）入口。`run` 只负责 E1；断点续跑检查**源配置哈希、原始 sim 输出和状态**，不能仅看 `run_status.json` 存在就跳过。
2. `simulink/paper/evaluate_E1_dense.m`：复用 `evaluate_paper_runs` 的四元数误差逻辑及统一成功门槛，额外输出真值迎角/预测越界区别；代码应可对保存的 `sim_output.mat` **重新分析而不用重跑**。
3. `simulink/paper/report_E1_dense.m`：导出完整状态矩阵、A/B 成对表、失败原因、统计图和中文 Markdown 报告。
4. `simulink/paper/check_E1_dense.m`：验收与保护测试（见 §8）。

**不要**把原 `run_paper_suite('all',...)` 扩展成自动跑 144 例，也不要在 E1 完成时调用会改写旧四组报告的 `paper_finish` / `paper_final_report`。如果复用旧函数需要小修改，必须保证其他 E2/E3/E4 不退化，记录改动原因。

执行时注意 MATLAB/Windows 的现实问题：`mlab.cmd` 是当前仓库已有启动器；模型/参考 bank 可能因 MATLAB 内存映射或文件句柄无法覆盖，应为**每例使用唯一 `reference_bank.mat` 路径**并在关闭模型后处理下一例，必要时分进程运行。不要在运行循环中写共享 `simulink/data/online_plan_bank.mat`。每一组都以源 bank 构建独立 A/B 参考输入，不得偷偷修改源 bank。

### 实验矩阵构建伪代码（必须落实为可执行 MATLAB）

```matlab
angles = [5 7.5 10 12.5 15 17.5 20 22.5 25];
times  = [2 4 6 8 10 12 16 20];
for i = 1:numel(angles)
  for j = 1:numel(times)
    pair_id = sprintf('E1D_%04d_%04d',round(angles(i)*10),round(times(j)*10));
    for has_cmg = [false true]
      % 由同一源 bank、同一参考曲线创建 case 输入，生成独立 case 模型。
      % build_paper_case -> build_closedloop(..., explicit bank_path)
      % sim(Simulink.SimulationInput(...,'CaptureErrors','on'))
      % 保存原始数据和每例状态；失败独立保留，继续剩余任务。
    end
  end
end
```

此处 ID 会产生 `0050/0020` 等稳定编码；以实际 `case_name` 规则确保 Simulink 模型名 ≤63 字符。若评估器出现异常，原始输出仍须保留并标注 `ANALYSIS_FAIL`，不许把分析失败伪装成 `CONTROLLER_FAIL`。

## 6. 绘图与统计口径（论文可直接使用）

### 主图一：A/B 双面板可行域热力图

- 两面板轴范围、格子大小和颜色语义**完全一致**，横轴规定过渡时间 [2,4,6,8,10,12,16,20] s；纵轴俯仰参考增量 [5,7.5,...,25]°。
- 颜色/符号至少区分 **通过、跟踪不达标、控制器失效、模型适用域外、运行环境失败**。
- 图下注明“离散闭环任务通过区域，非数学理论可达域；有效域外不等同于不可达”。
- 如需展示有效域遮罩，可用独立纹理或清晰图例，不能把越界格子和跟踪失败涂成同一种颜色。

### 主图二：最短已测试通过参考过渡时间 vs 俯仰幅值

- 对每个幅值，在已测试且满足 `SUCCESS` 的时长中取最小值；没有则标记 `not_observed`，**不写 0、不擅自外推**。
- 如果最短测试时长 2 s 已成功，只能写“≤2 s（在本扫描下）”或“最短已测试通过为 2 s”，**不可宣称理论最短 2 s**。
- 最小成功时间在非单调情况下也按离散网格忠实报告，并注明扫描分辨率限制。

### 补充图：边界附近的动力学解释

选取**事先明确的规则**而不是挑好看的点：至少一对“仅舵面不达标、CMG 成功”、一对“两者均成功”、一对有效域外（若存在）。对于成功案例展示参考/实际姿态、误差、舵面指令和饱和、CMG 力矩/框架速率；对于越界案例重点展示真值迎角和模型断言触发时刻。

### 报告至少给出四类统计

1. 全 72 对网格中，A/B 各自的 `SUCCESS`、`TRACKING_FAIL`、`MODEL_DOMAIN_FAIL`、`CONTROLLER_FAIL`、`INFRA_FAIL` 数量（分母 72，五类加总应为 72）。
2. **共同有效且可比较的成对工况数量**，以及该子集中的 A/B 通过数量、A 失败/B 通过的增益格子数量、A 通过/B 失败的退化格子数量。
3. 按幅值列出最短已测试通过过渡时间（A、B 和比较差），只用真实通过数据。
4. 有效域外工况清单：幅值、过渡时间、arm、中断时刻、触发源、最后有效迎角/高度/速度；区别预测越界和真值越界。

不要把“有效域外”剔除后仍写“72 对中成功率”；使用**共同有效配对**时明确新的分母。不要把当前较少的通过点等同于不可物理实现。

## 7. 可选的边界加密（第二阶段，不能影响主网格结论）

主网格完整并生成初版统计后，沿某幅值、时长的一格成功/失败边界，仅在**两臂物理适用域内**增加局部样本，例如俯仰每 1°或过渡时间每 1 s。建议先设预算**最多 24 对（48 次仿真）**，由程序根据明确规则决定位置并记录 `exploratory=true`、选择原因；若边界样本不满足可比较条件，则不加密、不用强行补齐预算。

**主网格与边界补充分别统计、分别画图或明确标记**，不允许用事后补充点改变主网格的既定分母或阈值。若总运行资源不足，**主 144 次优先，可选加密可以不做**。

## 8. 验收要求（不接受“脚本能跑”代替实验完成）

### 运行前

- 校验当前 Git commit 和分支；记下未提交更改；对两个正式 `.slx`、默认 `online_plan_bank.mat` 做 SHA256 快照。
- 确认 `build_paper_case` 真实启用 A/B 相同证书模式、相同 F20 内环、相同 4 SQP 与 0.2 s；只在 B 模型有 CMG；固定步长为 0.001 s。
- 先用 5°/20 s 和 20°/4 s 各自 A/B 共 4 例做冒烟测试，核对参考、日志、断言和重跑行为；**正式矩阵仍须包含它们，不可因为冒烟而减少 144 例**（可在完全同配置、同哈希、同输出可追溯的前提下复用这些原始结果）。

### 运行后

- **主矩阵必须有 144 个任务状态记录，72 个配对 ID**；每例有 `case_config.json`、`run_status.json`、来源信息和保存的结果/失败堆栈；至少保存成功及可评估中断例的原始 Simulink 输出。
- 失败不能只返回字符串；必须单独记录物理模型越界/预测越界/控制异常/环境异常，保持可追溯。`INFRA_FAIL` 必须修复和重试，或明确宣布本批次未完成，不计入物理结论。
- 用与运行代码分离的**重算入口**重新读取 `sim_output.mat`，验证状态与统计数值一致；抽查 A/B 同配对参考四元数、角速度、初始机体/飞行状态和扰动，保证差异数值近机器精度。必要时做全部 72 对自动核查。
- 对照数据不得包含不同有效运行时长的 RMSE 直接排名；只有完整 60s 的 RMSE 才列入主 A/B 对比。
- 正式两 `.slx` 和默认 bank 的 SHA256 与实验前相同；所有物理和空气动力限制仍启用；没有改其他三组实验原始数据。
- 输出 `acceptance.json`，明确 `matrix_completed`、`cases_total`、`paired_cases`、`comparability_pass`、`protected_inputs_unchanged`、`domain_diagnostics_complete`、`analysis_reproducible`、`paper_E1_acceptance_complete`，**每项有真实证据路径**。未满足则为 `false`，不能凭批处理退出码 0 宣称全通过。

### 提交要求

- 只把新增/修改的 `.m`、必要的 `.ps1`、`.md`、CSV 小表、PNG/PDF 汇总图、JSON 摘要与验收结果纳入 Git；不要把 144 例大型 `sim_output.mat` 全部推送仓库（保留本机）。
- 提交中文 commit，并给出修改文件列表、实际运行次数、失败类型统计、主要 A/B 发现、所有未完成项。
- 不在本任务中自动合并到 `main`；不覆盖原 `docs/simulink/论文四组实验_实施与结果.md`，新报告独立命名为 `docs/simulink/实验一_密集姿态机动可行域_实施与结果.md`。

## 9. 操作入口与交付格式

优先实现简单、稳定的 MATLAB 入口，示例：

```matlab
addpath('simulink');
init_paths;
addpath('simulink/paper');
run_E1_dense('preflight','20261105_E1_dense_v1');
run_E1_dense('smoke','20261105_E1_dense_v1');
run_E1_dense('run','20261105_E1_dense_v1');
run_E1_dense('analyze','20261105_E1_dense_v1');
run_E1_dense('report','20261105_E1_dense_v1');
check_E1_dense('20261105_E1_dense_v1');
```

以上函数名和参数需要 Codex 实际实现，不能只把这些示例命令放进 README。使用仓库现有 `simulink/mlab.cmd` 调起 MATLAB；不得要求用户手动修改 144 份参考文件。提供一条可重复执行的 PowerShell 或 MATLAB 入口，以及单例失败的可重试入口；如果任务规模大，可以脚本分批运行，但**每批必须即时持久化和留日志，直到所有 144 例状态完整**。

**最后交付**：新代码、144 例真实运行状态（以及可选补充）、两张主要论文图、成对对比 CSV、完整中文报告、验收 JSON/MD。结果必须基于真实仿真，**不得预设 CMG 一定成功、不得人工补齐缺失数据、不得把有限离散网格称作理论可达域**。
