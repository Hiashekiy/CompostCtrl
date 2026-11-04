# Simulink/MPC 实验踩坑记录

> 来源：`experiment/simulink-mpc-five-sqp` → `direct-servo-mpc` → `direct-servo-sqp-acceptance-diagnosis` → `fast-torque-authority` 四轮实验中的真实故障。
> 每条包含：**症状 → 根因 → 正确做法**。最后有「开工检查清单」。

## 1. MATLAB 启动（最贵的一次坑，白等 9 分钟）

**症状**：`matlab.exe -batch "..."` 启动后进程存活、CPU 接近 0、stdout 无任何输出，300 s 超时仍在跑。

**根因**：DSH 文件沙箱只允许写工作区，MATLAB 无法写 `%APPDATA%\MathWorks` 的配置文件，启动阶段挂住（不是崩溃，不会自己退出）。

**正确做法**：永远用仓库自带启动器
```powershell
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
.\simulink\mlab.cmd "addpath('simulink'); init_paths; <函数名>"
```
`mlab.cmd` 把 `APPDATA/LOCALAPPDATA/MATLAB_PREFDIR/TMP/TEMP` 全部重定向到仓库内，并设置 `MW_MINGW64_LOC`。

**附加代价**：每次 `mlab.cmd` 启动 40–60 s。**把互不依赖的检查合并到一次 `-batch` 调用**（例：`check_fast_authority_equivalence; benchmark_fast_authority(400)`）。我因此浪费了约 15 分钟。

## 2. Windows 文件锁（三处，都会让整轮实验白跑）

| 症状 | 根因 | 正确做法 |
|---|---|---|
| `save` 报 `无法写入文件 A_single_call.mat: Invalid argument` | 刚退出的 MATLAB 进程仍持有句柄（仓库 `build_closedloop.save_with_retry` 注释里写了这条） | 写新目录；或 `try/catch` + 延时重试 |
| `exportgraphics` 报 `PNG 导出失败: Could not open file` | 同上，重写**历史结果目录**里的 PNG | 不要重写历史目录，只往新目录写图 |
| `git checkout -- <file>` 报 `unable to unlink old ...: Invalid argument` | 同上 | 关掉占用的 MATLAB，或先 `git stash` 再处理 |

**通用教训**：报告/统计脚本一律「只读历史目录、只写新目录」。`report_five_sqp_comparison.m`、`report_fast_authority.m` 都是这么写的；`report_online_authority_closedloop.m` 会重写全部历史目录，正是它连崩两次。

## 3. Simulink 模型名与加载

- **模型名长度上限 63 字符**。`NearSpace_CMG_FlightControl_SurfaceOnly_DSACC_hard_stop_fixed_A_2s`（67）直接报 `Simulink 模型名称无效`。→ 模型名用短码（`FC_AF_B_60`），结果目录再用长描述名。
- `Simulink.SimulationInput(m)` 在**模型未加载/不在路径**时同样报「模型名称无效」，极具误导性。`build_closedloop` 用 `case_name` 会把模型存到 `simulink/cases/`，必须显式加载：
```matlab
load_system(fullfile(root,'simulink','cases',[m '.slx']));
```
- `bdroot(b.BlockHandle)` 在 S-function `Terminate` 里拿到的名字可能不可靠，直接用它拼文件名会写出乱码文件名（`ׁ_direct_authority.mat`）。→ 在 `Start` 里 `get_param(bdroot(...),'Name')` 缓存下来。

## 4. Simulink 日志读取

- 根 Outport 的 Dataset **名字不可依赖**：`yout.get('component_1')` 报 `Did not find any Dataset element`。用**位置索引** `yout{k}`（按端口顺序），并先 `setModelParameter('SaveOutput','on','SaveFormat','Dataset')`；`numElements<3` 时判失败。
- `squeeze(ts.Data)` 得到的朝向不要判两次：`values()` 内部已经做过一次「行数≠样本数就转置」，外面再判一次会把 `N×D` 转成 `D×N`，随后 `x(a,5:7)` 越界。→ **方向归一化只写一处**。
- `flight_conditions` 只有 3 列时不一定有 `beta`，`fc(a,1:4)` 会越界。→ 先判列数再补默认值。

## 5. MATLAB 语言陷阱（每次都烧掉一整轮）

1. **`/` 是 mrdivide，不是逐元素**：`wk*2*w./delta_scale.^2` 写成 `/`，当 `delta_scale` 是 3×1 时结果变 **1×3**，随后 `s.r(1:3,k)=rs.*U(1:3,k)` 变成 3×3 → `左侧 3×1、右侧 3×3`。**向量除法一律 `./`**。
2. **`diag(标量)` ≠ `diag(3×1)`**：`eye(3)-g` 在 `g` 是标量时报维度错。→ `g=diag(gain*ones(3,1))`。
3. **字符串里混 `.'`**：`'truth_total',truth.total.','model_total'` 被解析成「转置 + 多余引号」，报 `字符向量未正确终止`。→ 拆开写，或用 `truth.total(:)'` 并用临时变量。
4. **`orderfields(A,B)` 要求字段集合完全一致**，否则 `strcmp` 尺寸不匹配。→ `names=union(...)`，逐结构补 `NaN`，再 `cell2mat`。
5. **`[].field` 报「不支持使用点索引」**：S-function 首拍传进来的 `state=[]` 会炸在 `state.nu_previous`。→ 函数入口先建默认 struct。
6. **`jsondecode` 单元素返回 char、多元素返回 cellstr**。→ `if ischar(x),x={x};end`。
7. **`sprintf` 处理路径会把 `\a`/`\l`/`\y` 当转义**：`sprintf('...\ablation\legacy_%s_60s',arm)` 报 `转换字符 '\y' 无效`。→ `strrep(template,'%s',value)`。
8. **`if nargin<3 || isempty(state)` 必须在访问 `state.*` 之前**，否则默认值逻辑永远走不到。
9. **`assert` 才是门槛，`fprintf` 不是**。早期 `check_direct_servo_mpc` 只把雅可比偏差 `worst` 记录下来不判阈值，被审查直接点出「不是合格的验收」。→ 每个门槛都要 `assert` + 打印实测值。
10. **`rng(7)` 会重置而非保存状态**。→ `state=rng(7); ...; rng(state);`。

## 6. 控制与实验设计上的坑（比代码坑更贵）

1. **只看耗时会把「早停」当成「加速」**：直接舵机 A 组均值 52.6 ms（比生产 77.7 ms 快），实际是 **301 个周期里 297 个零接受、只有 4 次首控变化**，等于冻结指令。→ 必须同时统计 `accepted_steps`、首控变化周期数、`early_stop_reason` 分布。
2. **QP 线性化工作点用错单位**：把舵机指令（rad）当气动力矩（N·m）送进力矩级 `native_A_*/native_B_*`，QP 自认能把代价降到 0，非线性代价却不动。→ 线性化必须在**真实力矩工作点** `τ_nat+B_δ·δ` 处求，并用有限差分逐列核对。
3. **验收函数与 QP 目标不一致**：QP 有 CMG 正则、验收没有；差分项缺少 `k+1` 反向梯度。→ 抽一个 `*_objective.m` 作为**唯一**目标，控制器与诊断共用，并用方向导数 FD 断言梯度一致（<1e-6）。
4. **取消「代价必须下降」≠ 更安全更快**：取消后 A 组 RMS 14.9°→5.6° 看起来变好，但目标 J 从 76 涨到 8400、B 组周期 451 ms、超 200 ms 达 277/301 周期。→ 报结论要同时给精度、预算、J 趋势。
5. **单次 60 s 不能下结论**：legacy B 臂自身两次运行 220 ms 与 306 ms 最大值、超预算 1 与 4。→ 至少 2–3 次重复，且交错运行减少机器负载差。
6. **重跑会覆盖上一次结果**：按固定 tag 命名输出目录（`analytic_full_B_60s`），第二次重复运行直接覆盖第一次样本。→ 重复实验加 `_rep2` 后缀。
7. **不要跨提交比因果**：`d236358` 的结果不能当 `33f36f7` 的基线。→ 同一提交、同机、同条件重跑 legacy 作为唯一基线。
8. **解析证书不得伪造数值**：不执行二分就不能把 `max_residual_rad` 写成 0，也不能编造 8 条逆命令。→ 新增 `commands_evaluated=false`、`certified`、`endpoint_margin`，并让 `verify_authority_candidate` 按阶段选用对应证据；legacy 回退阶段按自身残差判。

## 7. Git 与仓库

1. **push 在本沙箱里必然失败**：`sh.exe: couldn't create signal pipe, Win32 error 5` + `could not read Username for 'https://github.com'`（凭据助手无法启动）。→ 本地提交后由用户在自己终端 `git push -u origin <branch>`。
2. **一次提交 188 MB 二进制**（10 个 `*_closedloop.mat` 20–25 MB + 10 个 `*_authority.mat` 9 MB）。→ 只提交 `summary.json`、`run_config.json`、`iteration_diagnostics.csv`、CSV/PNG 等文本与小文件；`.mat` 留在本地。
3. **工作区可能有别人未提交的改动**（本次就遇到 DT01/DT02 实验）：先 `git diff > tmp/…patch` + 复制未跟踪文件备份，再原样提交留痕，**绝不** `reset --hard` / `clean` / 强制切换。
4. **建议的提交粒度**：S0/S1/S2/S3 分开。我中途一次 `git commit --amend` 把分阶段提交折叠成一个，事后不好回溯。
5. 三份 `.slx` 模型被重新构建会显示为修改（正常，属可复现产物）。

## 8. 开工检查清单（照做可省一半时间）

- [ ] `git status --short`、`git branch --show-current`、`git rev-parse --short HEAD`；有他人改动先备份并原样提交。
- [ ] 确认实验条件：`dt_ctrl`、`max_sqp_iterations`（本系列固定 4 轮）、认证模式、是否动生产模型。
- [ ] 用 `simulink\mlab.cmd`，并把互相独立的检查合并成一次 `-batch`。
- [ ] 先跑「快门槛」（单元/等价性/2 s 冒烟），过了再上 60 s；失败要 `assert` 而不是打日志。
- [ ] 输出只写新目录；重复实验加后缀；报告表格从 JSON/CSV 生成，不手抄。
- [ ] 每次长实验后立刻 `diagnose/report` 汇总，顺手确认 `fallbacks`、`accepted_steps`、`over_200ms`、`analytic_fallback_count` 是否异常。
- [ ] 提交前确认没把大 `.mat` 加进索引；push 交给用户终端。
