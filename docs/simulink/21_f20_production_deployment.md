# 正式 Simulink 部署：F20 力矩闭环 + analytic_full 认证

> 部署脚本：`simulink/verify/deploy_f20_production.m`（本轮新写，方案里提到的同名配套脚本未随附件交付，按方案 §1–§4 的门槛自行实现）
> 被更新的文件：`simulink/models/NearSpace_CMG_FlightControl.slx`（**只有这一个生产模型**）
> 基线：`experiment/torque-margin-feedback-ablation` @ `a0f2406` → 分支 `experiment/production-f20-integration`
> 本次部署的所有数值都来自实际 MATLAB 执行，没有手抄；`.slxc` / `slprj` 是缓存，未编辑、未提交。

## 1. 部署的配置

| 参数 | 值 | 核验来源 |
|---|---|---|
| `flight` / `enabled` | true / true（在线飞行 B 组，舵面 + CMG） | 模型内 `Control_split` 为 2 输出 Demux，`Native_Plant/CMG` 存在 |
| `authority_mode` | `online_predicted` | 模型工作区 `native_controller_template.authority_mode` |
| `certificate_mode` | `analytic_full` | 同上 `authority_certificate_mode`（实测 6622 个阶段全部 `analytic_box`） |
| `torque_shrink_ratio` | 0 | `authority_torque_shrink_ratio` / `native_torque_shrink_ratio`（**不启用 M10**） |
| `torque_inner_mode` | `feedback_predictive` | `native_inner_mode` |
| `sqp_iterations` | 4 | `max_sqp_iterations`（未改成 5） |
| 上层控制周期 | 0.2 s | `dt_ctrl` / `native_control_period` |
| 下层力矩反馈周期 | 0.02 s | `native_inner_period`、`Previous_Allocation_Command` 采样时间 |
| 舵机 / 物理积分 | 0.001 s | 模型 `FixedStep`、`Surface_Actuator/Actual_state` 采样时间 |
| 外加恒定扰动力矩 | 未添加 | 仍由 `online_plan_bank.mat` 的物理扰动表驱动 |

生成指令就是方案第 2 节那一条（`case_name=''` 才会写 `simulink/models/`）：

```matlab
build_closedloop(60,true,true,'','online_predicted',false,4,.2,'analytic_full',0,'feedback_predictive');
```

控制器一行代码都没有重写：沿用已验证的 `online_shrink_certified_torque_box`（α=0 比特级直通）、`online_authority_rollout`、`online_fast_torque_box` / `online_fast_servo_certificate`、`online_servo_endpoint`、`online_torque_feedback_step`、`native_torque_feedback_allocation_sfun`。F20 分支的语义与实验组完全一致：`Torque_feedback_allocation` 替换 `Surface_control_allocation`，`Previous_Allocation_Command` 采样改为 0.02 s（**没有**偷偷换回 0.2 s 缓存，也没有额外修正）。

## 2. 启动器与备份

- MATLAB 一律用 `simulink\mlab.cmd`（直接 `matlab.exe -batch` 在本沙箱会卡在 AppData 配置）。
- 部署前按时间戳新建独立备份，**没有**复用 `frozen_models/` 里的旧备份：

```
备份目录：tmp\deploy_f20_backup\20261105_115816\
备份文件：NearSpace_CMG_FlightControl.before.slx   3,930,140 字节
SHA256  ：34A440C3CECD13CA0E1E4F9E9EC5E832C36A536CC0C68555ABDA5CCF6AB9DC78
```

（`tmp/` 被 gitignore，所以备份不会被误提交；需要长期保存请自行拷出。）

## 3. 门槛与实测证据

脚本把每一步都写成 `assert`，失败即停并回滚。四个门槛：

| 门槛 | 内容 | 实测 |
|---|---|---|
| **G1 preflight** | 当前生产模型必须还是「旧 0.2 s 开环分配 + 旧证书路径」 | `bytes=3930140 stop=60 step=.001 sqp=4 dt=0.2 nx=11 N=22 legacy_block=1 cmg=1`，`authority_certificate_mode=<字段不存在>`（即运行时走 `legacy`），`inner=<absent>` → 通过 |
| **G2 隔离等价** | 用非空 `case_name='DEP_F20CHK'` 单独构建并跑 60 s，与存档 `F20_B_60s` 逐点比较 | `commands` 差值 **0.000e+00**、`states` 差值 **0.000e+00**，60001 采样 / 301 上层拍 / 3001 下层拍 / 上下层回退 0/0 → 通过 |
| **G3 结构** | 重新生成的正式模型的块与模型工作区 | 见第 4 节 → 通过 |
| **G4 验收** | **正式模型文件本身**（从 `simulink/models/` 关闭后重新打开）跑 2 s 与 60 s 并与存档比较 | 2 s：`cmd 0.000e+00 / state 0.000e+00`，2001 采样 / 11 拍 / 101 拍；60 s：`cmd 0.000e+00 / state 0.000e+00`，60001 采样 / 301 拍 / 3001 拍，回退 0/0，姿态 RMS **0.6627°**、峰值 **2.2713°** → 通过 |

G2 通过才允许触碰正式 `.slx`；G4 失败会自动把备份写回并核对校验和。

## 4. 正式模型的结构核验（G3 输出）

```
Torque_feedback_allocation        FunctionName = native_torque_feedback_allocation_sfun
                                  4 个输入全部有连线、输出有连线
输入来源                          Control_split（tau_cmd 前馈）
                                  Aerodynamic_Surface（实测 delta_actual）
                                  Environment_And_Forces（实测 air = q̄,V,α,β）
                                  Native_Plant（实测 omega）
旧 Surface_control_allocation      已不存在（无激活分支）
Previous_Allocation_Command       SampleTime = 0.020 s
CMG 速率指令                       Control_split/2 → Native_Plant（未改）
CMG 子系统                        Native_Plant/CMG 存在
物理步长                          FixedStep = .001
舵机状态步长                      Surface_Actuator/Actual_state = 0.001 s
模型工作区                        analytic_full / feedback_predictive / shrink 0 / sqp 4 / 0.2 s / 0.02 s
                                  native_controller_template、reference_window、disturbance_schedule 均在
```

`git diff --stat` 确认**只有** `simulink/models/NearSpace_CMG_FlightControl.slx` 变化（3,930,140 → 3,929,264 字节），其它生产模型、CMG 模型、气动参数、历史结果都没有改。

## 5. 验收指标（正式模型上的 60 s 运行）

| 项 | 实测 | 要求 |
|---|---|---|
| 上层拍数 | **301** | 301 |
| 下层拍数 | **3001** | 3001 |
| 1 ms 采样 | **60001** | 60001 |
| 与存档 F20-B 数值差 | `commands` 0.000e+00 / `states` 0.000e+00 | 复现 F20-B |
| 姿态 RMS / 峰值 | **0.6627° / 2.2713°** | ≈0.6627° / 2.2713° |
| 三轴偏角绝对最大值 | 0.436332 / 0.349066 / 0.349066 rad（= 各轴 `delta_max`） | 逐轴 ≤ `delta_max` |
| 三轴速率绝对最大值 | 1.0472 / 1.0472 / 1.0472 rad/s（= `delta_dot_max`） | 逐轴 ≤ `delta_dot_max` |
| 信号有限性 | 全部有限，时间戳严格一致（`commands.Time == tau_actual.Time`） | 必须 |
| 上层安全回退 | 0 | 0 |
| 下层安全回退 | 0 | 0 |
| 代数环 | 无（2 s 与 60 s 均正常编译运行） | 无 |
| 关闭 / 重新加载 / 独立仿真 | 是（验收就是关掉模型后从 `simulink/models` 重开后跑的） | 必须 |

力矩层指标（**不作为验收条件**，如实记录）：三轴 `cmd_rms` = 1.0497 / 5.6490 / 1.5528 N·m，`err_rms` = 0.21062 / 0.73646 / 0.18466 N·m（逐轴相对 20.06 % / 13.04 % / 11.89 %），合成误差 RMS **0.4549 N·m**。与 `docs/simulink/20` 的 F20-B 数值完全一致，也就是**气动力矩跟踪并不优于 B0**（B0-B 合成 0.3913 N·m）。部署理由只是姿态跟踪与下层周期，不是力矩误差。

## 6. 计算耗时（同步仿真，**不等于**硬件实时保证）

正式模型先后跑过两次 60 s（本机负载不同，两次都记录，不挑好看的）：

| 运行 | 项 | 均值 | P50 | P95 | P99 | 最大 | 超周期 |
|---|---|---|---|---|---|---|---|
| 原子部署 `20261105_122145` | 上层 MPC（301 拍） | 58.2 ms | 47.7 ms | 124.5 ms | 199.9 ms | **279.2 ms** | >200 ms：**3** 拍 |
| 同一次 | 下层力矩反馈（3001 拍） | 0.49 ms | 0.42 ms | 0.93 ms | 1.70 ms | 2.78 ms | >20 ms：**0** 拍 |
| 早一次独立验收 `20261105_121226` | 上层 MPC（301 拍） | 50.3 ms | 45.8 ms | 85.7 ms | 111.3 ms | 147.8 ms | >200 ms：**0** 拍 |
| 同一次 | 下层力矩反馈（3001 拍） | 0.47 ms | 0.40 ms | 0.77 ms | 1.21 ms | 38.3 ms | >20 ms：**1** 拍 |

两次的控制量**逐位相同**（都是 0 差值复现 F20-B），差别只在机器负载：同一份 60 s 运行，上层 P95 从 85.7 ms 变成 124.5 ms、>200 ms 的拍数从 0 变成 3。**所以不能说「上层 MPC 稳定满足 200 ms 周期」**，只能说两次实测均值 50–58 ms、P95 86–125 ms、最坏拍可达 279 ms。`docs/Simulink实验踩坑记录.md` §6.5 早就记录过同类波动（legacy-B 两次运行最大值 220 ms vs 306 ms）。

- 每次上层每拍平均 QP 数 2.07、平均接受步 1.09、迭代上限 4（未出现 5 轮）。
- 下层 3001 拍的内迭代最多 4 次，1741/3001 拍命中指令变化限幅；内层预测力矩残差 RMS 0.733 N·m。
- 下层极值：原子部署那次最大 2.78 ms（全部 ≤20 ms，首拍 0.74 ms）；早一次出现一个 38.3 ms 的解释器/GC 抖动拍（P99 仍 1.21 ms）。这**不能**当成硬件实时性证据。

## 7. 如实记录的边界

1. **F20 的气动力矩跟踪不优于 B0**（合成 0.4549 vs 0.3913 N·m）。本部署不宣称「力矩跟踪最优」。
2. **同步仿真耗时不是实时性保证**，未做任何硬件在环或代码生成验证。
3. F20 组把两件事捆在一起：20 ms 反馈律 + `Previous_Allocation_Command` 改为 t⁻ 指令语义。这是方案明令要求保持的既有语义，本轮未拆分归因（`docs/simulink/20` §9 限制 1）。
4. 验收为**单次** 60 s 运行（+ 单次 2 s），未做重复运行统计。
5. 旧模型的 `authority_certificate_mode` 字段不存在 → 运行时默认 `legacy`；本次部署把它显式改成 `analytic_full`。二者数学等价已由 `docs/simulink/19` 逐位确认，本次 G2/G4 的 0 差值复现也再次印证。
6. 部署过程中出现过两次**脚本自身**的缺陷（`values()` 只声明一个输出、`tau_cmd` 未截到前三列），已修好并重跑；第一次回滚之所以失败是因为模型仍在本会话中打开、文件被锁，脚本已改为「先关闭再恢复」。为让审计链干净，最终流程是：恢复原始 `.slx` → 一次会话内跑完 G1→G2→G3→G4。

## 8. 回滚

```powershell
cd D:\项目\飞轮
Copy-Item 'tmp\deploy_f20_backup\20261105_115816\NearSpace_CMG_FlightControl.before.slx' `
          'simulink\models\NearSpace_CMG_FlightControl.slx' -Force
# 校验（应为 34A440C3CECD13CA0E1E4F9E9EC5E832C36A536CC0C68555ABDA5CCF6AB9DC78）
(Get-FileHash 'simulink\models\NearSpace_CMG_FlightControl.slx' -Algorithm SHA256).Hash
git checkout -- simulink/models/NearSpace_CMG_FlightControl.slx   # 或直接回退到 a0f2406
```

脚本自身也实现了自动回滚（异常时先用备份覆盖、再核对 `Simulink.getFileChecksum`），失败时会写 `rollback.json` 并把损坏版本另存为 `NearSpace_CMG_FlightControl.failed.slx`。

## 9. 复现与文件

```powershell
cd D:\项目\飞轮
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
# 仅快照当前正式模型
.\simulink\mlab.cmd "addpath('simulink'); init_paths; deploy_f20_production('check')"
# 只做隔离等价（不碰正式模型）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; deploy_f20_production('verify')"
# 完整部署（G1→G2→G3→G4，含自动回滚）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; deploy_f20_production('deploy')"
# 只对现有正式模型做验收（不重建）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; deploy_f20_production('accept')"
```

结果目录 `simulink/results/deploy_f20/<时间戳>/`：`deployment_record.json`（全部门槛数值）、`isolated_60s/`、`production_2s/`、`production_60s/`（含 `production_60s_timings.json`）。
