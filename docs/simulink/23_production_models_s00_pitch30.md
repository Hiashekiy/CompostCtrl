# 两个正式模型切换到 S00_pitch30（正向 30° 俯仰）场景

> 提交基线：`8d7fa35`（`experiment/production-f20-integration`）
> 脚本：`simulink/verify/switch_scenario_to_s00.m`
> 结果：`simulink/results/scenario_switch/20261105_s00/{A,B}/`
> 备份：`tmp/scenario_switch_backup/20261105_140658/`（切换前的两个 `.slx`，可逐字节回滚）

## 1. 改了什么、没改什么

只把两个正式模型里**烘焙的参考机动**换成任务 1 = `S00_pitch30`（`configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_{no_cmg,with_cmg}.yaml`：俯仰 +30° hold，轨迹 110–130 s，窗口 90–150 s），**每个模型各自的控制器配置保持原样**：

| 模型 | 控制器配置（切换前后一致） | 参考机动 |
|---|---|---|
| `simulink/models/NearSpace_CMG_FlightControl.slx`（B，舵面+CMG，11 状态） | `analytic_full` 证书、力矩盒收缩 0、F20 20 ms 力矩闭环（`feedback_predictive`/0.02 s）、`Previous_Allocation_Command` 0.02 s、4 轮 SQP、0.2 s、1 ms 物理 | A4 90 s 参考 → **S00_pitch30（301 点 / 60 s，俯仰 12.09°→28.62°@38.0 s）** |
| `simulink/models/NearSpace_CMG_FlightControl_SurfaceOnly.slx`（A，仅舵面，7 状态） | `legacy` 证书、旧 0.2 s 开环分配器、零盒收缩、0.2 s、1 ms 物理 | 同上 → **S00_pitch30** |

生成方式仍是生产构建指令，只有 bank 换掉（`case_name=''` 才写 `simulink/models/`）：

```matlab
% B
build_closedloop(60,true,true ,'','online_predicted',false,4,.2,'analytic_full',0,'feedback_predictive');
% A
build_closedloop(60,true,false,'','online_predicted',false,4,.2,'legacy',0,'legacy');
```

## 2. 构建门槛（全部 assert，输出见 `switch_report.json`）

| 项 | B | A |
|---|---|---|
| 文件大小 | 2,783,357 字节 | 2,699,824 字节 |
| 证书模式 | `analytic_full` | `legacy` |
| 力矩盒收缩 | 0 | 0 |
| 下层模式 / 周期 | `feedback_predictive` / 0.020 s | `legacy` / 0.200 s |
| `Previous_Allocation_Command` | 0.020 s | 0.200 s |
| 反馈块 / 旧分配器块 | 有 / 无 | 无 / 有 |
| CMG 子系统 | 有 | 无 |
| 物理步长 / StopTime | 0.001 / 60 | 0.001 / 60 |
| 烘焙参考 | 301 点、0–60 s、俯仰 12.09°→峰值 28.62°@38.0 s，与导出 bank 的 q_ref 差 <1e-12 | 同 |

## 3. 切换后的实测（真跑，不是推断）

| 模型 | 2 s 冒烟 | 60 s 机动窗口(0–50 s) RMSE | 窗口峰值 | 全程(0–60 s) RMSE | 俯仰力矩误差 RMS | 偏角饱和（三轴最大） | 上层拍数 | 安全回退 |
|---|---|---|---|---|---|---|---|---|
| **B**（+CMG，F20） | 0.0002° | **0.3092°** | 0.768° | — | 0.7382 N·m | 30.1 %（滚转） | 301 | 0 |
| **A**（仅舵面，0.2 s 分配） | 0.0001° | **3.9430°** | 8.324° | 3.8254° | 1.0348 N·m | 81.9 %（滚转） | 301 | 0 |

- A 的 60 s 数值与 `docs/simulink/22` 里 S00_pitch30 的 A 臂**逐位一致**（3.9430° / 8.324° / 1.0348 N·m / 81.9 %），说明这次切换在数值上就是「同一任务、同一配置」，没有引入别的变化。
- B 用 F20（生产部署态）得 0.3092°，而 `docs/simulink/22` 里 B 用旧 0.2 s 分配器得 0.3119° —— 在本任务上 20 ms 力矩闭环几乎不改变姿态结果（与 `docs/simulink/20` 的结论一致）。
- 两臂都是 301 个上层拍、0 次安全回退、无 NaN、无物理越界、无代数环。

## 4. 需要知道的三件事

1. **仓库默认参考库没动**：`simulink/data/online_plan_bank.mat` 已恢复为 HEAD 的 A4 90 s 参考（它是"下一次构建的输入"，不是模型运行时的依赖；两个 `.slx` 已经各自把 S00 参考烘焙进模型工作区 `native_reference_bank` / `reference_window`）。
2. **要重建出同样的 S00 场景**，先把存档的 bank 拷过去再跑构建：
   ```powershell
   Copy-Item simulink\data\scenario_banks\S00_pitch30_with_cmg_bank.mat simulink\data\online_plan_bank.mat -Force   # B
   Copy-Item simulink\data\scenario_banks\S00_pitch30_no_cmg_bank.mat   simulink\data\online_plan_bank.mat -Force   # A
   ```
   否则默认 bank 会把场景悄悄改回 A4 参考。两个 bank 已入库存档（各 1.2 MB）。
3. **A 组工作区多了 3 个惰性变量**：`native_inner_mode='legacy'`、`native_inner_period=0.2`、`native_torque_shrink_ratio=0`，模板里也显式多了 `authority_certificate_mode='legacy'` 与 `authority_torque_shrink_ratio=0`。这些字段在原 A 模型里是"缺失→运行时默认"，现在显式写出，**数值等价**（第 3 节的 3.9430/1.0348 与旧配置逐位相同即为证据）。A 组仍然没有 F20 反馈块，仍是旧 0.2 s 分配器。

## 5. 复现

```powershell
cd D:\项目\飞轮
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
# 每个模型一个进程（bank 文件在 MATLAB 内被内存映射，同进程内不能反复覆盖）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; switch_scenario_to_s00('B','20261105_s00')"
.\simulink\mlab.cmd "addpath('simulink'); init_paths; switch_scenario_to_s00('A','20261105_s00')"
git checkout -- simulink/data/online_plan_bank.mat   # 恢复默认参考库
```

## 6. 回滚

```powershell
Copy-Item 'tmp\scenario_switch_backup\20261105_140658\NearSpace_CMG_FlightControl.before.slx' `
          'simulink\models\NearSpace_CMG_FlightControl.slx' -Force
Copy-Item 'tmp\scenario_switch_backup\20261105_140658\NearSpace_CMG_FlightControl_SurfaceOnly.before.slx' `
          'simulink\models\NearSpace_CMG_FlightControl_SurfaceOnly.slx' -Force
# 或直接 git checkout -- simulink/models
```

切换前的 SHA256：B `68D3BF1A3F8F5911B7FACC8F8B749712B34042981E185C25CABD7A7BE8334E18`，A `F450FB3C3DDA0739F4F54422E13AA91598F708FF01FED6B1C1FF4B53796DFEEB`。
