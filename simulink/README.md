# 原生 Simulink 迁移交付入口

后续开发请优先阅读完整的[开发接续手册](../docs/simulink/原生Simulink迁移_开发接续手册.md)，包含模块接口、代码、修改与重建步骤、Simulink联动和已知缺口。

本入口是原生迁移的唯一 Simulink 实现。早先的 S08 Python/TCP 移植（`simulink/README.md`、`s08_startup.m`、
`open_case.m`、`run_case.m`、`blocks/s08_*`、`py/ctrl_server.py` 等）已在迁移完成后删除，历史版本从提交历史取。

在 MATLAB R2024a 中：

```matlab
cd('D:\项目\飞轮');
addpath('simulink');
init_paths;
open_system('simulink/models/NearSpace_CMG_FlightControl.slx');
% 点击 Run。A 组：NearSpace_CMG_FlightControl_SurfaceOnly.slx。
```

运行依赖 Aerospace Blockset/Toolbox、Simulink、本机 `D:/Tools/hpipm` 和 `D:/Tools/blasfeo`。
MEX 为 Windows x64。其他平台需运行 `build_hpipm_mex` 与 `build_prediction_mex` 重建。

## 目录结构

根目录只放入口，其余按角色分层：

```text
simulink/
├── init_paths.m          入口：把 modelgen/verify/data/models/hpipm 加进路径并初始化 HPIPM
├── repo_root.m           仓库根（避免每个脚本按目录深度数 fileparts）
├── mlab.cmd              headless 启动（重定向 MATLAB 的 APPDATA/PREFDIR/TMP 到仓库内）
├── README.md
├── modelgen/             生成脚本：产出 models/ 与 cases/ 下的 SLX、公开力模型（build_*）
├── verify/               验证与报告：check_*、report_*、finalize_delivery、probe_stage1
├── hpipm/                控制器：在线 RTI、QP 组装、Level-2 S-Function、HPIPM MEX 与生成的预测 C
├── models/               组件 SLX（CMG/气动/舵机/被控对象）+ 3 个默认闭环模型
├── cases/                16 个 P0 工况 SLX（A/B × S00–S07，由 modelgen/ 生成）
├── data/                 参数、参考、大气与公开力数据
├── py/                   Python 导出与批量回归入口
├── tests/                验证夹具（QP 对照数据、6DOF 验证模型）
└── results/              运行证据（native_stage1/plant/actuators/cmg/closedloop/delivery）
```

`addpath('simulink')` 之后 `init_paths` 会把 `modelgen/`、`verify/`、`models/`、`hpipm/`、`data/` 一起加进路径，
所以脚本一律按函数名调用；模型按全路径 `load_system` 打开（默认模型在 `models/`，工况模型在 `cases/`）。
命名约定：目录表示角色，文件名表示"动词+对象"（`build_plant.m`、`check_actuation.m`），不再加 `native_`；
`hpipm/` 里的 `native_mpc_sfun`、`native_rti_step`、`native_F_B` 等保留原名，因为它们已写进已保存模型的
S-Function 块参数和预测 MEX 文件名，改名需要重建全部模型并重编 C/MEX。

## 模型定义

- `models/NearSpace_CMG_FlightControl*.slx`：公开升阻力替代模型、MSIS2 查表、重力、真实舵机、CMG 驱动、在线 MATLAB RTI/HPIPM 的工程闭环。
- `cases/NearSpace_CMG_AttitudeValidation*_S0*.slx`：原有 P0 总气动力矩命令接口；不接平动力，用于姿态实验。P0 CMG 直接跟随框架速率命令；P1 保留驱动环节。
  （早先那份不带工况后缀的 `models/NearSpace_CMG_AttitudeValidation.slx` 没有内嵌模板与参考库、无法运行，已删除。）
- 无 CMG A 组使用独立的 7 状态/3 控制预测模型与不含 CMG 的原生真值结构；B 组使用 11 状态/7 控制预测。
- Case 模型在 ModelWorkspace 内嵌控制模板与未来参考，防止切换模型后复用别的工况。求解器代码/MEX 仍通过 init_paths 加路径。
- 工程飞行模型另有一个 `Reference_Manager` 子系统：把内嵌参考库按控制时刻压平成"参考窗口"信号，
  **接到控制器 `Online_RTI_HPIPM` 的第 2 个输入端口**（端到端可见的参考通路）；同一子系统还引出姿态/角速度/
  名义力矩三路信号，接到日志与 `Attitude_vs_Reference` 示波器。控制器不再读模型工作区，库只在构建与首拍自检时使用。
  详见手册 §6.4 与 §7.1。

所有物理部分使用可检查的工具箱/基础模块。两个自定义 Level-2 MATLAB S-Function 仅负责 MPC 算法及舵面控制分配。
生成的 C/MEX 预测模型仅用于控制器预测与导数，不承担真值刚体或执行器动力学。

## 重现验证

```matlab
build_stage1;
check_integrations;
check_native_allocation;
check_native_assembly;
check_native_rti;
check_actuation;          % P0 B 组：框架角必须真的移动（惰性执行机构的回归门）
check_layout;             % 目录分层自检：脚本可解析、模型在 models|cases
report_p0_matrix;         % 由已有日志重算 16 组 P0 指标表
report_closedloop;        % 工程飞行模式的跟踪指标 JSON
plot_p0_ab;               % 八组 P0 的 A/B 四面板对比图（P0_<case>_AB.png + metrics.csv）
plot_flight_ab;           % 工程飞行 A/B 数据对比图（读 60 s 日志，不重跑仿真）
```

Python 原始参考导出与批量实验入口在 `simulink/py/`。八组 A/B 回归入口为 `python simulink/py/run_native_p0_matrix.py`，支持断点续跑。
单工况快速复核（不写进度缓存）：`check_p0_case('S00_pitch30',true)`。

## 交付边界

公开升阻极曲线是工程替代，不是当前飞行器的辨识结果；侧向系数假设为零，推力为零，代表无动力滑翔。
当前为直接姿态参考模式；没有完成自主位置/速度制导、实时部署、代码生成或真实飞行器全包线认证。
八组 P0 A/B 指标与历史 Python 同量级；P0 B 组曾因框架驱动接线丢失而整段失效，已修复并加回归门，详见 `docs/simulink/08_closed_loop_validation.md`。
名义未来环境与实际平动的预测更新、ABC 机制实验、鲁棒性实验及 GUI 参数截图的状态见 `docs/simulink/08_closed_loop_validation.md` 和最终验收报告。
