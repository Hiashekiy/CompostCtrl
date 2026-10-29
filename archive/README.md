# archive —— 旧项目归档（历史参考，不是新实验结果）

本目录是"增量式 CMG 构型路径搜索"新实验启动前，对旧项目（近空间稀薄大气飞行器
气动舵面 / CMG 协同姿态控制）的归档快照。归档对象是工作区 `main` 分支
HEAD `6fcc0f99221bcbb5eccf26c007e19295a47964f0`（2026-10-29）时的内容。

> **警示：本目录中的一切结论、指标与图件都属于旧方法。**
> 它们不得被引用为新实验（A/B/C/D 四组、构型路径搜索）的结果，
> 新实验的所有输出只能写入 `experiments/cmg_incremental_search/results/`。

## 1. 归档策略

| 内容 | 处理方式 | 理由 |
|---|---|---|
| `nearspace/`、`scripts/`、`legacy/`、`tests/`、`configs/`、`config.yaml`、`README.md`、`requirements.txt`、`.gitignore` | 全量复制到 `legacy_source/` | 体积小（约 3 MB），必须能脱离 git 直接阅读 |
| `docs/*.md`、`docs/references/AST-V1.pdf` | 全量复制到 `legacy_reports/` | 旧结论与建模底稿需要原文留存 |
| `results/`（512 个时序/指标/元数据文件，约 47.9 MB） | **只登记不复制** | 逐文件记录 SHA-256 与来源提交，避免同一仓库重复存储近 50 MB |
| `figures/`（18 张 PNG，约 5.0 MB） | **只登记不复制** | 同上；且可由旧脚本重绘 |
| 环境 | `legacy_environment/`：`pip-freeze.txt`、`requirements.txt`、`git-and-runtime.txt` | 记录 HEAD、解释器与依赖版本 |
| `tmp/`（7 个草稿）、`__pycache__/` | 不归档 | 被 `.gitignore` 排除或为构建产物 |

选择混合策略是为了让 `archive/` 可校验、可追溯，同时不制造"同一份历史数据在仓库里
存两遍"的歧义风险。若确实需要自包含快照，可执行
`python tools/archive_legacy.py build --include-results`（会先要求清空本目录）。

## 2. 清单与校验

* `ARCHIVE_MANIFEST.csv` —— 逐文件清单：`original_path, archive_path, category,
  archived, size_bytes, sha256, tracked_in_git, git_blob, purpose`。共 643 行。
* `ARCHIVE_STATS.json` —— 构建统计（643 个文件 / 58.74 MB，其中复制 113 个 / 5.81 MB）。
* `ARCHIVE_INVENTORY.md` —— 人类可读盘点：类别统计、运行与测试入口、已知问题。

复算全部哈希并与清单比对：

```bash
python tools/archive_legacy.py verify
# 期望输出：checked=643 mismatches=0
```

单文件核对（PowerShell）：

```powershell
(Get-FileHash -Algorithm SHA256 archive\legacy_source\nearspace\cmg.py).Hash.ToLower()
```

交叉核对归档副本 == HEAD 中的对象：

```powershell
git hash-object archive\legacy_source\nearspace\cmg.py
git rev-parse HEAD:nearspace/cmg.py     # 两者应相同
```

取回只登记未复制的结果数据：

```bash
git checkout 6fcc0f99 -- results figures
```

**已知限制（换行符）**：清单中的 `sha256` 是**当前工作区字节**的哈希。本仓库
`core.autocrlf=true` 且没有 `.gitattributes`，因此在 Windows 上重新 clone 后，文本文件
可能被检出为 CRLF，字节级 SHA-256 会与清单不同。此时以 `git_blob` 列为准做交叉核对
（`git rev-parse HEAD:<path>`），它与换行符无关。若需要字节级可复现，应在后续阶段
统一加 `.gitattributes`（`* -text`），但这会改变整个仓库的检出行为，属于决策项，本阶段
不擅自引入。

## 3. 归档没有做什么

* 没有移动、删除、改写任何旧文件；工作区旧代码与历史结果仍在原位。
* 没有执行 `git clean -fd`、重置、批量删除或历史重写。
* 没有把旧代码接入新实验链路；新实验独立实现，不 import `nearspace/`、`legacy/`。
* 没有修复旧文件里的编码损伤（例如 `legacy/README.md` 第 1 行的字面 `\n\r\n`），
  因为改写会让清单哈希失去"原样"意义；缺陷在 `ARCHIVE_INVENTORY.md` §6 记录。

## 4. 阶段 0 验收自检

- [x] 旧代码、配置、结果、图表、日志和报告已清点（643 个文件全部进入清单）
- [x] 哈希与文件数量校验通过（`verify`：643/643 一致，复制件与原件的 SHA-256 逐字节比对）
- [x] 未提交改动得到保护（归档前唯一未跟踪文件是新的指导实施文件，未纳入归档，单独提交）
- [x] 历史结果未覆盖（`results/`、`figures/` 未被写入，仅登记）
- [x] 归档单独提交（`chore: archive legacy CMG experiments`）
