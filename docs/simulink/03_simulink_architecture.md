# 原生迁移当前结构与后续接口

2026-11-02。保留历史 S08 S-Function 目录和结果，仅供回归对照；新增原生模型放入 `simulink/models/`。

## 当前实施状态

| 对象 | 状态 |
|---|---|
| MATLAB/库路径/许可证审计 | 已记录；Aerospace 实例化阻塞 |
| 6DOF 原生验证 | 脚本已编写，未执行成功 |
| 参数字典与继承链溯源 | 已导出首个 S00 B 基线 |
| CMG 几何映射 | 原生 SLX，67 个静态样本测试通过 |
| CMG 驱动和框架积分 | 待实现 |
| 气动力矩参考数据 | 96 个 Python 样本，分解检查通过，未生成原生气动模型 |
| HPIPM MATLAB/MEX | 已定位本机源码目录，未编译或求解 |
| 位置速度外环、完整气动力、主模型、闭环 A/B | 待实施 |

## 下一步可独立推进

1. 将 CMG 几何图封装为显式 gamma、nu 输入和 H、A、tau 输出；驱动动态独立搭建，不藏入几何函数。
2. 实现参数化气动力矩和舵面执行机构，按 `aerodynamic_reference.csv` 做逐点验证；气动力数据不足时明确记录缺口。
3. 在仓库内部构建 HPIPM MEX 输出。官方本机目录为 `D:/Tools/hpipm/interfaces/matlab_octave`，依赖 BLASFEO；官方 Simulink S-Function 在 `interfaces/simulink/hpipm_sfun.c`，需要配合生成的 QP 数据 C 文件，不能直接等同于通用动态 OCP-QP 接口。
4. 从真实 RTI 导出固定 QP，建立矩阵、控制序列、目标和残差的 Python/MATLAB 对照。
5. Aerospace 许可恢复后，先完成原生 6DOF 解析验证，再接入已独立验证的各子系统。

## 信号约定

物理子系统保留机体系自然力矩、舵面增量力矩、CMG 框架反作用力矩和 omega×H 修正的独立日志。
总力矩只合成一次。P0 直接力矩模式与 P1 真实舵机模式显式区分，不把 P0 tau_a 当成偏角。
预测层保持当前姿态研究范围，不能自动把位置速度加入 MPC 目标。
