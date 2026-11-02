# 原生 Simulink 迁移：阶段 0 审计

日期：2026-11-02。分支：`experiment/simulink-native-flight-control`。

## 当前结论：第一阶段被许可证阻塞

MATLAB 实际启动成功，版本为 **24.1.0.2537033 (R2024a)**。
从本机库加载并确认了 `aerolib6dof/6DoF (Quaternion)`，不是同名自建子系统。
但在 `add_block` 复制这个原生模块时，MATLAB 拒绝签出 Aerospace Blockset 许可。
单独执行 `license('checkout','Aerospace_Blockset')` 返回 0，诊断为：

```text
License checkout failed.
License Manager Error -5
Cannot find a license for Aerospace_Toolbox.
Feature: Aerospace_Toolbox
Licensing error: -5,357.
```

初步 `license('test',...)` 的成功结果不能作为可用许可的证据；以后以实际签出和模型执行为准。
这是 MATLAB 的实际返回信息，不据此推断许可证文件应如何修改。没有修改许可证、替代原生模块或绕过产品检查。

## 可复查产物

- `simulink/results/native_stage1/environment.txt`：版本、库加载与参数输出。
- `simulink/results/native_stage1/installed_products.csv`：已安装产品及版本。
- `simulink/results/native_stage1/licenses.csv`：各产品 test、checkout 和原始诊断。
- `simulink/results/native_stage1/library_6dof_metadata.json`：真实库路径、库版本、掩膜参数与端口数量。
- `simulink/native_stage1_probe.m`：重新执行审计的入口。

## 已定位模块

| 模块 | 实际库路径 | 证据状态 | 计划 |
|---|---|---|---|
| 6DoF (Quaternion) | `aerolib6dof/6DoF (Quaternion)` | 库可加载；实例化时签出失败 | 第一阶段指定刚体模块 |
| COESA Atmosphere Model | `aerolibatmos/COESA Atmosphere Model` | 库中已列出；未仿真 | 后续先检查高度适用范围 |
| ISA Atmosphere Model | `aerolibatmos/ISA Atmosphere Model` | 库中已列出；未仿真 | 待审定是否使用 |
| Simscape Multibody | `sm_lib` | 顶层库可加载；具体关节及惯量模块未逐项核查 | 第二阶段机械验证候选 |
| 姿态变换、Lookup Table、Transfer Fcn、Rate Limiter、Saturation、Matrix Multiply | 待逐项记录 | 尚未完成完整端口和许可核查 | 后续补齐 |

`aerolib` 和 `aeroblks` 不是本机可加载的顶层库名称，不能照搬旧环境脚本里的猜测。
未完成 Library Browser GUI 定位、参数对话框截图与端口截图；以上属于程序化库查询证据，不冒充 GUI 截图。

## 6DoF 接口与参数

输入 1：机体系三轴力，N。输入 2：机体系三轴力矩，N·m。

| 输出端口 | 库内部 Outport 标签 | 含义 |
|---|---|---|
| 1 | Ve (m/s) | 地面坐标系速度 |
| 2 | Xe (m) | 地面坐标系位置 |
| 3 | Euler (rad) | 欧拉角 |
| 4 | DCM | 方向余弦矩阵 |
| 5 | Vb (m/s) | 机体系速度 |
| 6 | p,q,r (rad/s) | 机体系角速度 |
| 7 | pdot,qdot,rdot (rad/s²) | 机体系角加速度 |

注意：该库变体没有直接四元数输出端口。测试准备读取内部积分状态，而不是由 DCM 重构、归一化后声称积分四元数范数合格。

掩膜参数：`xme_0`、`Vm_0`、`eul_0`、`pm_0`、`Mass`、`Inertia`、`k_quat`。
库默认初始状态全零，质量 1，惯量 eye(3)，四元数归一化增益 1。验证脚本使用专门解析算例质量 10 kg、惯量 diag([2 3 4]) kg·m²；这些不是项目飞行器参数。

## 下一步前置条件

在本机 MATLAB 中恢复可正常签出的 Aerospace Blockset 及其报告缺少的 Aerospace Toolbox 许可。
之后重跑审计和 `build_native_stage1`，完成原生模型、解析对照、模型清单与 GUI 证据。
本阶段尚未验收，不进入气动/舵面建模。
