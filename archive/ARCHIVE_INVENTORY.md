# 归档清单（ARCHIVE INVENTORY）

> 本文件与 `ARCHIVE_MANIFEST.csv`、`ARCHIVE_STATS.json` 均由
> `python tools/archive_legacy.py build` 自动生成，请勿手工编辑；
> 手工说明写在 `archive/README.md`。

## 1. 盘点对象

- 仓库根目录：`D:\项目\飞轮`
- 分支：`main`
- HEAD 提交：`6fcc0f99221bcbb5eccf26c007e19295a47964f0`
- 盘点时间（UTC）：2026-10-29T17:00:11+00:00
- 纳入清单文件数：643（58.74 MB）
- `git status --short --branch`：

```
## main...origin/main
?? docs/CMG增量式构型路径搜索_实验指导实施文件.md
?? tools/
```

## 2. 归档策略

混合式：源代码、配置、测试、文档/报告、环境元数据**全量复制**到 `archive/`；
生成的 `results/` 与 `figures/` **只登记不复制**，逐文件记录 SHA-256 与所在提交。
历史结果与代码在原位保持不变，归档过程不移动、不删除、不重写任何旧文件。

## 3. 按类别统计

| 类别 | 文件数 | 字节数 | 是否复制 | 说明 |
|---|---:|---:|---|---|
| `code_current` | 39 | 465,148 | 是 | 当前有效代码（nearspace 实现与 scripts 运行入口），新实验只作参考，不直接依赖 |
| `code_legacy` | 40 | 2,530,355 | 是 | 已被取代的早期实现（反作用轮/俯仰通道/第一代 CMG），仅可追溯性保留 |
| `config` | 16 | 44,186 | 是 | YAML 配置：飞行器/大气/控制器权重/实验矩阵 |
| `data` | 512 | 47,942,875 | 登记 | 历史结果原始时序/指标/元数据（只登记不复制） |
| `doc` | 1 | 12,195 | 是 | 仓库说明文档 |
| `environment` | 2 | 141 | 是 | 依赖与忽略规则，复现环境依据 |
| `figure` | 18 | 4,992,875 | 登记 | 历史图件（只登记不复制，可由旧脚本重绘） |
| `reference` | 1 | 2,485,907 | 是 | 参考文献原文（PDF） |
| `report` | 12 | 242,996 | 是 | 旧项目报告与实施方案（历史结论，不得冒充新实验结果） |
| `tests` | 2 | 27,801 | 是 | 旧项目单元测试（无 pytest 依赖，脚本式自检） |

## 4. 目录布局

```text
archive/
  legacy_source/      nearspace/ legacy/ scripts/ tests/ configs/ config.yaml README.md requirements.txt .gitignore
  legacy_reports/     旧项目报告与参考资料（docs/*.md、docs/references/AST-V1.pdf）
  legacy_results/     csv/ figures/ logs/ metrics/ 骨架 + README 指针（见策略）
  legacy_environment/ pip-freeze.txt requirements.txt git-and-runtime.txt
  ARCHIVE_INVENTORY.md  本文件
  ARCHIVE_MANIFEST.csv  逐文件清单：原路径/归档路径/字节数/SHA-256/来源提交
  ARCHIVE_STATS.json    构建统计
  README.md             人工说明与取回方式
```

## 5. 运行入口与测试入口（旧项目）

运行入口（`scripts/`，见原 `README.md`）：

- `scripts/check_horizon_sensitivity.py`
- `scripts/check_maneuver_feasibility.py`
- `scripts/e1_bias_ablation.py`
- `scripts/e2_actuator_allocation.py`
- `scripts/e3_qp_forensics.py`
- `scripts/evaluate_c1_cmg_ab.py`
- `scripts/evaluate_cmg_ab.py`
- `scripts/evaluate_cmg_maneuver.py`
- `scripts/plot_cmg_ab.py`
- `scripts/plot_cmg_maneuver.py`
- `scripts/plot_device_sweep.py`
- `scripts/reproduce_c1.py`
- `scripts/run_all_experiments.py`
- `scripts/run_c1_cmg_ab.py`
- `scripts/run_cmg_ab.py`
- `scripts/run_cmg_maneuver_ab.py`
- `scripts/run_device_sweep.py`
- `scripts/validate_cmg_actuator.py`
- `scripts/validate_maneuver_feasibility.py`
- `scripts/verify_cmg_weights.py`

旧实现入口（`legacy/`，已不在实验链路内）：

- `legacy/run_all.py`
- `legacy/run_cmg.py`
- `legacy/run_cmg_compare.py`
- `legacy/run_three_axis.py`

测试入口（脚本式自检，无 pytest 依赖，在工作区根目录运行）：

- `python tests/test_atmosphere_real.py`
- `python tests/test_cmg_ab.py`

## 6. 已知问题与不可复现项（转述 `legacy/README.md`，未擅自修复）

- 早期 RW / CMG 图件生成脚本（`make_figures.py` 等 5 个）在旧归档移动时按 ANSI 编码
  读写导致中文字符串损坏，已删除；其 PNG 仍在 `results/legacy_rw/`、`results/cmg_ab/baseline_legacy/`，
  重绘须按 `scripts/plot_cmg_ab.py` 风格重写。
- `results/cmp_A.csv`（145 s、仅舵面、100 km 顶点脉冲）在整理前被一次试运行覆盖并删除；
  同类工况由 `results/cmg_ab/disturbance/A/` 复现。
- `legacy/README.md` 第 1 行含字面转义符（`n`r`n`），是旧归档过程中的编码损伤；
  为保证哈希可核验，本归档保留原样未改写。
- 旧结果采集时的解释器版本未记录；当前环境见 `legacy_environment/`。

## 7. 未纳入归档的内容

- 被 `.gitignore` 排除的临时草稿 `tmp/`：7 个文件
  - `tmp/AST-V1.txt`
  - `tmp/commit_msg.txt`
  - `tmp/commit_msg2.txt`
  - `tmp/dbg_lp.py`
  - `tmp/snap_qp.py`
  - `tmp/snap_ref.py`
  - `tmp/tag_hist.py`
- `docs/CMG增量式构型路径搜索_实验指导实施文件.md`：新实验（阶段 2 起）的指导实施文件，属于新流程，不是旧项目产物
- `__pycache__/`、`*.pyc`：解释器缓存，构建产物，不具归档价值。
- `tools/archive_legacy.py`：本归档工具本身，属于新流程而非旧项目。

## 8. 历史结果溯源

- `results/` 与 `figures/` 由提交 `6fcc0f99221bcbb5eccf26c007e19295a47964f0` 跟踪；清单中每行 `git_blob` 是该文件在
  该提交中的对象哈希，可与 `git cat-file -p` 交叉核对，用于确认工作区内容与提交一致。
- 归档不改变任何历史结果文件，`verify` 会重新计算清单中每个文件的 SHA-256 并逐一比对。

## 9. 校验方式

```bash
python tools/archive_legacy.py build     # 生成清单与副本（已完成）
python tools/archive_legacy.py verify    # 重新计算哈希并与清单比对
# 任意单文件：
#   PowerShell: (Get-FileHash -Algorithm SHA256 <path>).Hash.ToLower()
```

## 10. 未提交改动保护

构建时未跟踪但已纳入清单的文件：

- `results/cmg_ab/_cache/msis2_table_150.csv`（31101 B）
- `results/cmg_maneuver/_cache/msis2_table_150.csv`（31101 B）
