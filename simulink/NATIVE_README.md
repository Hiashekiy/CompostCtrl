# 原生 Simulink 迁移交付入口

后续开发请优先阅读完整的[开发接续手册](../docs/simulink/原生Simulink迁移_开发接续手册.md)，包含模块接口、代码、修改与重建步骤、Simulink联动和已知缺口。

本入口与旧 `simulink/README.md` 的 S08 Python/TCP 版本分开。原生主链不依赖运行 Python 控制服务器。

在 MATLAB R2024a 中：

```matlab
cd('D:\项目\飞轮');
addpath('simulink');
native_startup;
open_system('simulink/NearSpace_CMG_FlightControl.slx');
% 点击 Run。A 组：NearSpace_CMG_FlightControl_SurfaceOnly.slx。
```

运行依赖 Aerospace Blockset/Toolbox、Simulink、本机 `D:/Tools/hpipm` 和 `D:/Tools/blasfeo`。
MEX 为 Windows x64。其他平台需运行 `build_hpipm_mex` 与 `build_prediction_mex` 重建。

## 模型定义

- `NearSpace_CMG_FlightControl*.slx`：公开升阻力替代模型、MSIS2 查表、重力、真实舵机、CMG 驱动、在线 MATLAB RTI/HPIPM 的工程闭环。
- `NearSpace_CMG_AttitudeValidation*.slx`：原有 P0 总气动力矩命令接口；不接平动力，用于姿态实验。P0 CMG 直接跟随框架速率命令；P1 保留驱动环节。
- 无 CMG A 组使用独立的 7 状态/3 控制预测模型与不含 CMG 的原生真值结构；B 组使用 11 状态/7 控制预测。
- Case 模型在 ModelWorkspace 内嵌控制模板与未来参考，防止切换模型后复用别的工况。求解器代码/MEX 仍通过 native_startup 加路径。

所有物理部分使用可检查的工具箱/基础模块。两个自定义 Level-2 MATLAB S-Function 仅负责 MPC 算法及舵面控制分配。
生成的 C/MEX 预测模型仅用于控制器预测与导数，不承担真值刚体或执行器动力学。

## 重现验证

```matlab
build_native_stage1;
check_native_integrations;
check_native_allocation;
check_native_assembly;
check_native_rti;
```

Python 原始参考导出与批量实验入口在 `simulink/py/`。八组 A/B 回归入口为 `python simulink/py/run_native_p0_matrix.py`，支持断点续跑。

## 交付边界

公开升阻极曲线是工程替代，不是当前飞行器的辨识结果；侧向系数假设为零，推力为零，代表无动力滑翔。
当前为直接姿态参考模式；没有完成自主位置/速度制导、实时部署、代码生成或真实飞行器全包线认证。
名义未来环境与实际平动的预测更新、ABC 机制实验、鲁棒性实验及 GUI 参数截图的状态见 `docs/simulink/08_closed_loop_validation.md` 和最终验收报告。
