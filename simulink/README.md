# Simulink 移植（S08 MIL）

本目录把 S08 的飞行器/CMG/控制器闭环搬到 Simulink，并给出**可核对的等价性证据**。

| 文档 | 内容 |
|---|---|
| `docs/torque_gap/simulink_port_spec.md` | 架构、数据契约、坐标符号、环境适配、调试记录、复现命令 |
| `results/simulink_port/report.md` | 验收结果（C1/C2/C3/C4 表 + 混沌本底归因） |
| `results/simulink_port/runs/*.slx` | 每次运行**生成**的模型（由 `run_case.m` 重建，不入手工编辑） |
| `figures/simulink_port/` | 逐工况对比图 + 汇总图 |

## 三条命令跑通

```bash
python simulink/py/export_for_simulink.py --all        # 1. 从 Python 参考模型导出工况与黄金数据
simulink\mlab.cmd "addpath('simulink'); check_plant_step"   # 2. C1 单步等价性（需设 MATLAB_HOME）
python simulink/py/run_closedloop.py --all             # 3. 闭环（起 Python 控制器 + 跑 MATLAB）
```

## 打开/重跑某个已生成的模型（手动操作）

在**你自己的 MATLAB 桌面**里（不要用 `mlab.cmd`，那个是给无人值守 `-batch` 用的）：

```matlab
cd D:\项目\飞轮
run('simulink/s08_startup.m')                          % 加路径 + 列出模型
open_case('A4_descent_replay_S00_with_cmg_P0','replay') % 刷新该工况输入并打开模型
% 然后直接点 Run
```

闭环模型额外需要 Python 控制器在跑（另开一个终端）：

```bash
python simulink/py/ctrl_server.py --case configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg_P0.yaml --port 55901
```

然后 `open_case('A4_descent_replay_S00_with_cmg_P0','closedloop')` → Run。

> 为什么必须走 `open_case`：模型从**固定路径** `results/simulink_port/data/CURRENT_case.json`
> 与 `CURRENT_u.csv` 读输入（Level-2 S-Function 的 setup 拿不到其它通道），
> 直接双击 .slx 会复用"上一次运行"的工况快照；A 组是 7 状态、B 组是 11 状态，维度不符会直接报错。

## 关键结论（详见 report.md）

- **C1**：单步植物与 Python 差 1e-16（浮点结合律级）。
- **C3**：用录制的控制序列驱动 Simulink 植物，300 步状态差 ≤ 4e-10、姿态误差差 ≤ 1e-8 deg。
- **C4**：闭环（同一 Python RTI 控制器 over TCP + 同一植物）在非混沌工况下差 ≤ 1e-9；
  在 CMG 被推到 σ_min≈0.6 的 S00/S03-B 工况下，差异与**参考模型自身对 1e-9 扰动的响应**同量级
  → 属模型条件敏感性（混沌），不是移植误差。

## 已知边界

- 只用 P0 植物（11/7 状态）；P1（18 状态真实舵机）未移植。
- 控制器未编译成 C（本机无可用 C 编译器），以 Python 进程托管；
  装上 VS2022 Build Tools / MinGW 后可把 CasADi+HPIPM 生成的 C 编成 MEX，接口不变。
- 解释执行，单次 60 s 仿真 0.3–3 s（回放）/ ≈60–90 s（闭环，含 300 次 TCP 往返）。
