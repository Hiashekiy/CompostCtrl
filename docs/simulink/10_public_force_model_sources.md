# 公开气动力替代模型：来源与范围

用户于 2026-11-02 确认没有升阻力及推力模型，并授权查找公开来源。

## 筛选结果

MathWorks HL-20 示例确实提供六个气动系数，但官方说明其主要来源是初步亚声速构型风洞数据。因此不能直接用于本项目数千 m/s 高空场景并声称高超声速可信：
https://www.mathworks.com/help/aeroblks/nasa-hl-20-lifting-body-airframe.html

NASA 有 Mach 10 HL-20 风洞报告，覆盖代表性迎角约 2°–41°、侧滑 0°及 ±2°。这是特定缩比构型及雷诺数的数据，不是本项目飞行器的通用系数；本轮尚未完成图表数字化与误差审核：
https://ntrs.nasa.gov/citations/19990117251

## 首个可复现的工程替代极曲线

采用 JuMP 官方航天飞机再入示例中 Betts 基准的升阻系数公式；OpenMDAO/Dymos 官方再入示例作为独立建模背景。基准是无动力滑翔，不能由此虚构发动机推力。

- JuMP：https://jump.dev/JuMP.jl/stable/tutorials/nonlinear/space_shuttle_reentry_trajectory/
- Dymos：https://openmdao.github.io/dymos/examples/reentry/reentry.html

alpha_deg=180*alpha_rad/pi。

CL = -0.20704 + 0.029244*alpha_deg。

CD = 0.07854 - 0.0061592*alpha_deg + 0.000621408*alpha_deg²。

通过 `export_public_force_tables.py` 生成 -40°–40°、0.1° 间隔表，表断点存为 rad。负迎角包含在 Dymos 公开基准所允许的控制范围内；此处范围是本次工程保护区间，并不构成试验验证范围声明。初版只覆盖正迎角，在 13.92 s 触发保护后已修正。123 个正负迎角/侧滑角静态点最大力误差为 1.82e-12 N。
不复制原基准的英制质量、面积或大气常数来混入现有 SI 参数；采用这些系数与当前飞行器面积/质量结合时，必须称为**混合参数工程替代模型**。

局限：无 Mach/Re 依赖；本轮额外假设 CY=0；不包含真实侧向气动力、气动弹性、稀薄流修正或发动机性能。现有力矩模型仍需保留用于姿态研究，不据此声称力与力矩属于同一个经验证飞行器数据库。

这项工作补齐的是可追溯的替代输入，不能据此宣布真实飞行器全包线验证完成。后续应增加该替代模型参数敏感性及适用域越界诊断。
