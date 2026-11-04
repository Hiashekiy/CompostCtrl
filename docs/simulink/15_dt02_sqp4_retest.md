# 恢复 0.2 秒控制周期后的 60 秒实测

日期：2026-11-04。分支：`experiment/simulink-mpc-five-sqp`。MPC 与分配器恢复为 0.2 秒，最多 4 轮 SQP，物理积分保持 0.001 秒；两份生产 SLX 已保存该配置。

A/B 串行各运行 60 秒、301 个控制周期，均无回退。每轴气动力矩相对误差为 RMS(指令−实际)/RMS(指令)，无延迟对齐、不含 CMG 力矩；姿态误差为四元数之间的最小旋转角。

| 配置 | 滚转/俯仰/偏航相对误差 | 姿态 RMS/峰值（度） | MPC 平均/P95/最大（ms） | 超过 200 ms |
|---|---|---|---|---|
| A | 14.750% / 11.520% / 8.686% | 3.8250 / 8.3244 | 166.03 / 362.94 / 972.95 | 122/301 |
| B | 12.594% / 11.174% / 10.435% | 1.1098 / 3.6162 | 273.92 / 458.55 / 550.74 | 214/301 |

A 为无 CMG，B 为含 CMG。完整 MPC 计时包含预测、认证、QP 装配和求解，含首拍冷启动。

- A 原始结果：`simulink/results/online_authority/L2_60s_20261104_212212/`；HPIPM 内部单次 QP 平均 0.588 ms。
- B 原始结果：`simulink/results/online_authority/L2_60s_20261104_212322/`；HPIPM 内部单次 QP 平均 1.354 ms。

- 汇总：`simulink/results/online_authority/dt02_sqp4/comparison.json`、`comparison.csv`。
- 0.1 秒实验的原始汇总保持在 `dt01_sqp4/comparison.json`，没有被本次结果替换。

本轮存在大量超过 200 ms 的周期，不能判定满足 200 ms 实时预算。墙钟计时与本机运行状态有关；Simulink 同步仿真不会将求解超时自动转成控制延迟。

复现：

```matlab
addpath('simulink'); init_paths;
a=build_closedloop(60,true,false,'','online_predicted',true,4,.2);
b=build_closedloop(60,true,true,'','online_predicted',true,4,.2);
```
