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

# 基础闭环（B 组形态，无构型迁移参考），落盘原始时序与指标
python experiments/cmg_incremental_search/scripts/run_base_case.py
python experiments/cmg_incremental_search/scripts/run_base_case.py \
    --name delta5 --set scenario.maneuver.delta_deg=5.0
```

包结构用 `sys.path` 指向本目录后以 `src.*` 导入（测试与脚本都这样做），
因此从仓库根目录运行即可，无需安装。

## 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| 3 | 基础物理/CMG/分配器/闭环 + 六项测试 | 完成（8/8 测试通过） |
| 4 | 构型离散图 + 有界 A* 离线搜索 | 完成（6/6 测试通过） |
| 5 | 搜索接入分配器、单场景闭环 | 待做 |
| 6 | 历史路径热启动与局部修复 | 待做 |
| 7 | 可行性扫描、冻结主场景 | 待做 |
| 8 | A/B/C/D + 消融 + 多初始构型 | 待做 |
| 9 | 图表与报告 | 待做 |

阶段 3 的细节、数值与已知缺陷见
`docs/阶段3-基础模型与闭环实现报告.md`。
