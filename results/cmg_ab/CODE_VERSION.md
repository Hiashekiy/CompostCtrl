# 本结果集对应的代码版本

`results/cmg_ab/` 下的全部数据（18 次运行 + 权重核验 + 可行性表 + 指标表）
由下面这次提交所对应的工作树产生：

| 项 | 值 |
|---|---|
| commit | `62424d1`（"CMG collaborative attitude control: A/B experiment pipeline (plan V1.0)"） |
| Python | 3.14.7 |
| numpy / scipy / osqp | 2.5.3 / 1.18.1 / 1.1.3 |
| 大气后端 | NRLMSIS-2.0（pymsis，F10.7 = 150, Ap = 4, 45°N），查表缓存 `results/cmg_ab/_cache/` |
| 控制周期 / 积分步长 | 0.2 s / 0.005 s |
| 冻结权重 | `r_cmg_transient = 0.05`，`r_cmg_trim = 0.50` |

## 为什么 meta.json 里的 code_commit 是 "no-git-repository"

搭建这条流程时工作区还不是 git 仓库，`nearspace/sim.py` 的 `_git_commit()`
在运行时找不到 `.git`，因此每次运行的 `meta.json` 中 `code_commit` 记录为
`no-git-repository`。仓库是在结果产出**之后**才初始化的，上面这个 commit
就是当时工作树的快照。今后的运行会自动记录真实的 commit。

## 可复现性

仿真完全确定（无随机数、无随机初始化、OSQP 从固定 warm start 迭代），
因此按 README §3 的命令重跑会得到逐位相同的 `timeseries.csv`
（唯一例外是 `solve_time_ms` —— 它取决于机器负载）。

复现命令：

```bash
python scripts/run_all_experiments.py --jobs 5
```
