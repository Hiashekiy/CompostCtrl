# experiments/cmg_incremental_search

增量式 CMG 构型路径搜索：独立实现、独立数据、独立报告的实验目录。
指导实施文件见 `docs/CMG增量式构型路径搜索_实验指导实施文件.md`。

**硬约束**

* 不 `import` `nearspace/` 或 `legacy/`：物理、CMG、控制器全部在本目录重写并自测。
* 所有输出只写入本目录的 `results/`；`results/`（仓库根）与 `figures/` 属于历史结果，只读。
* 参数全部来自 `configs/*.yaml`；`search.max_expansions` 与 `search.max_search_time_ms`
  缺失即启动失败（`src/config.py:validate`），不允许默认无限搜索。

## 目录

```text
configs/            base.yaml / scenario_high_altitude_maneuver.yaml / search.yaml
data/               atmosphere_msis2_f107150.csv（与旧项目 results/_cache 同哈希的只读副本）
src/dynamics/       rigid_body / aerodynamic_surface / cmg_pyramid / vehicle_plant / atmosphere
src/control/        attitude_controller / control_allocator / safety_fallback
src/search/         构型图、未来需求、边评估、有界 A*、热启动、局部修复（阶段 4–6）
src/simulation/     reference / runner / logger / metrics / config
tests/              基础模型测试（无 pytest 依赖）
scripts/            运行入口
results/            raw / metrics / figures / reports
```

## 命令

```bash
# 基础模型测试（指导实施文件 §3.5 的六项 + 坐标约定）
python experiments/cmg_incremental_search/tests/test_base_models.py
# 构型图与有界 A* 离线测试（§13 阶段 2）
python experiments/cmg_incremental_search/tests/test_search_graph.py
# 热启动与局部修复测试（§13 阶段 3）
python experiments/cmg_incremental_search/tests/test_warm_start.py

# 基础闭环（B 组形态）
python experiments/cmg_incremental_search/scripts/run_base_case.py

# 单场景闭环（A/B/C/D），--scenario 可切到冻结主场景
python experiments/cmg_incremental_search/scripts/run_search_case.py \
    --scenario scenario_main.yaml --policy C

# 可行性扫描（§4.2）
python experiments/cmg_incremental_search/scripts/scan_feasibility.py

# 正式实验矩阵（A/B/C/D + 五项消融 × 5 个初始构型）
python experiments/cmg_incremental_search/scripts/run_matrix.py --only core
python experiments/cmg_incremental_search/scripts/run_matrix.py --only ablation

# 汇总表 + 全部 10 张图（只读原始日志）
python experiments/cmg_incremental_search/scripts/make_reports.py
```

包结构用 `sys.path` 指向本目录后以 `src.*` 导入（测试与脚本都这样做），
因此从仓库根目录运行即可，无需安装。

## 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| 3 | 基础物理/CMG/分配器/闭环 + 六项测试 | 完成（8/8） |
| 4 | 构型离散图 + 有界 A* 离线搜索 | 完成（6/6） |
| 5 | 搜索接入分配器、单场景闭环 | 完成 |
| 6 | 历史路径热启动与局部修复 | 完成（4/4） |
| 7 | 可行性扫描、冻结主场景 | 完成（`configs/scenario_main.yaml`） |
| 8 | A/B/C/D + 五项消融 + 5 个初始构型 | 完成（45 次运行） |
| 9 | 10 张自动生成图 + 结果报告 | 完成 |

**主场景**：100 km、俯仰 10°/20 s、起点 10 s、60 s 仿真、搜索步长 1.0 s、预算 60 ms。

**结果一句话**（详见 `results/reports/实验实施与结果报告.md`）：

* A（仅舵面）RMSE 1.582°；B（舵面+CMG）0.483°；C（每周期有界 A*）0.486° 且多花 75 ms/周期
  → **每周期重搜无收益**；
* D（历史复用+局部修复）0.456°、残差 −6.6%、`σ_min` 0.01→0.29 → **一致但很小的改善**，
  且消融显示收益来源更像"构型缓慢离开行程边缘"而非"未来需求引导"。

数据、图件与报告全部由脚本生成，不手工改数。

