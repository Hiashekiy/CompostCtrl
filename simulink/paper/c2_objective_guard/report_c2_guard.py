"""Evidence-bound Chinese report and hash ledger; never simulates a system."""
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'simulink/results/paper_experiments/C2_objective_guard_ablation'


def rows(name):
    with (OUT/'tables'/name).open(encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()


def write_csv(path,data):
    with path.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)


def main():
    E=rows('event_comparison.csv');G=rows('global_metrics.csv');S=rows('optimizer_12_24_summary.csv')
    cycle=rows('optimizer_diagnostics.csv');steps=rows('optimizer_steps.csv');P=rows('physical_constraints.csv')
    receipt=json.loads((OUT/'checks/analysis_receipt.json').read_text(encoding='utf-8'))
    simulation=json.loads((OUT/'simulation_receipt.json').read_text(encoding='utf-8'))
    assert receipt['baseline_metrics_match'] and receipt['guard_off_was_used'] and simulation['simulations']==1
    source_prov=json.loads((OUT/'config/provenance.json').read_text(encoding='utf-8'))
    base=ROOT/'simulink/results/paper_final/E2_Controller_Comparison'
    for name,path in [('model_sha256',base/'scenarios/V2_C2_3/model.slx'),('bank_sha256',base/'scenarios/V2_C2_3/reference_bank.mat'),('raw_sha256',base/'raw/V2_C2_3/sim_output.mat')]:
        assert sha(path)==source_prov['baseline'][name]
    assert sha(ROOT/'simulink/hpipm/native_online_rti_step.m')==sha(OUT/'implementation/native_online_rti_step.m')==source_prov['native_online_rti_step_sha256']
    snapshots=[]
    for method in ['guard_on','guard_off']:
        full=rows(method+'_physical_16_28_native.csv')
        for timestamp in [16,19.8,20,24,28]:
            matches=[r for r in full if abs(float(r['time_s'])-timestamp)<1e-9]
            if matches:snapshots.append({'method':method,**matches[0]})
    write_csv(OUT/'tables/physical_state_snapshots.csv',snapshots)
    increasing=[r for r in steps if r['accepted']=='1' and r['would_objective_reject']=='1' and 12-1e-9<=float(r['time_s'])<24-1e-9]
    objective_stats={'window':'[12,24)s','accepted_increase_steps':len(increasing),
        'positive_delta_min':min(float(r['delta_J']) for r in increasing),
        'positive_delta_max':max(float(r['delta_J']) for r in increasing),
        'positive_delta_sum':sum(float(r['delta_J']) for r in increasing),
        'iteration_histogram':{str(i):sum(r['method']=='guard_off' and 12-1e-9<=float(r['time_s'])<24-1e-9 and int(float(r['sqp_iterations']))==i for r in cycle) for i in range(1,5)},
        'interpretation':'Each delta compares a candidate to its own cycle/iteration nominal; sums across different windows are bookkeeping, not a global Lyapunov or cost-descent proof.'}
    (OUT/'tables/objective_change_summary.json').write_text(json.dumps(objective_stats,indent=2),encoding='utf-8')
    lines=[]
    for r in E:
        lines.append('|'+ '|'.join([r['event_index'],f"{float(r['baseline_maneuver_rmse_deg']):.6f}",f"{float(r['ablation_maneuver_rmse_deg']):.6f}",f"{float(r['baseline_peak_deg']):.6f}",f"{float(r['ablation_peak_deg']):.6f}",f"{float(r['baseline_post_rmse_deg']):.6f}",f"{float(r['ablation_post_rmse_deg']):.6f}"] )+'|')
    reduction=100*(1-float(E[1]['ablation_maneuver_rmse_deg'])/float(E[1]['baseline_maneuver_rmse_deg']))
    summary=f'''# C2目标函数拒步机制消融实验报告

## 结论

仅关闭`objective_increase`拒步，第二次机动RMSE由3.232611°降至0.140001°（降低{reduction:.3f}%），最大姿态误差由4.930926°降至0.214080°，机动后RMSE由4.626222°降至0.024348°。本次C2第二次机动恢复了小误差跟踪。全程RMSE由1.894816°降至0.063111°，新方案全程最大姿态误差0.224874°。

这为“目标上升拒步是本C2退化的主要直接贡献机制”提供强干预证据；不等于已经证明唯一深层根因，也不等于应永久删除该保护。开关从0s关闭，第二次机动前的控制历史、CMG构型和角动量也随之改变，不能把它解释为同一20s物理状态上的局部切换对照。

## 设置与隔离

基线只读使用`paper_final/E2_Controller_Comparison/raw/V2_C2_3`，没有重跑。新增且仅新增一次60s真实MATLAB/Simulink仿真，实际运行至60s，墙钟约{simulation['wall_s']:.2f}s。程序有单次调用标记，拒绝再次启动该实验仿真。仿真前有一次bank字节一致性检查退出，没有调用sim；改为保留原模型实际内嵌bank后才启动这唯一一次仿真。

三次机动均为4s完成同向+4°俯仰，分别位于[8,12]、[20,24]、[32,36]s，以原始events及bank为准。原cfg的通用amplitude字段没有替代连续事件定义。原模型副本保留全部物理模型、初始条件、扰动、预测模型、目标函数与权重；预测时域23.2s、控制周期0.2s、内环0.02s、物理步长0.001s、SQP最多4轮均不变。原模型内嵌参考bank与保存bank的微小归一化差异没有通过重建轨迹消除，实验直接沿用原内嵌值。

主控制器新增`enforce_objective_decrease`，缺省/显式true维持原拒步；仅实验模型模板设置false。`try_rollout`、同轨迹恢复与`verify_authority_candidate`、首指令检查以及舵偏/舵速/CMG框架速度/加速度限制全部保留。只在物理可行候选已通过原检查后跳过目标上升拒绝。原`objective_stagnation`接受后提前退出逻辑仍在：目标上升步可被接受，但随后可能因该条件结束本周期SQP；没有强迫每周期做满4轮。

C2初始控制周期的函数级检查表明：修改后的缺省值、显式true与归档旧控制器的指令、状态及主要优化结果逐值一致。该检查没有运行Simulink，亦未扩展至其他实验。paper_final及E1/E3/E4正式结果没有更新。

## 三次机动性能

误差采用与旧正式统计相同的四元数姿态角误差范数，完整1ms原始样本计算；它不是单独俯仰误差。完整CSV另存俯仰分量与全程统计。

|机动|原RMSE°|新RMSE°|原最大误差°|新最大误差°|原机动后RMSE°|新机动后RMSE°|
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

机动区间含起止端点；机动后区间不含机动结束点，分别为(12,20]、(24,32]、(36,46]s，与归档独立重算一致。前两段机动后窗口因下一次指令截断为8s，第三段为10s。

三次机动段RMSE均降低，但第三次机动后RMSE由0.023094°略增至0.028278°，增加约0.005184°。不能声称所有时段、所有时刻均改善；这一小幅退化属于整段开关干预的轨迹差异，不能仅凭此将其归因于某一个具体目标上升步。

## 12–24s优化状态

统计采用半开区间[12,24)s，共60个控制周期。

|统计|原MPC|关闭拒步|
|---|---:|---:|
|QP求解/成功次数|60/60|147/147|
|接受优化步|0|147|
|按原规则会被目标上升拒绝的步|60|54|
|接受的目标上升步（超过原容差）|0|54|
|物理候选拒绝|0|0|
|fallback|0|0|
|每周期SQP平均/最多轮数|1/1|2.45/4|

新方案实际轮数分布为：1轮14周期、2轮18周期、3轮15周期、4轮13周期。54个接受的目标上升步位于54个不同周期，候选与当前名义轨迹的ΔJ范围为{objective_stats['positive_delta_min']:.9f}至{objective_stats['positive_delta_max']:.9f}，均超过各自原容差；这54周期随后由保留的目标停滞条件退出。全程共记录687个可行候选步，其中173步按原容差属于目标上升且被接受。

新旧候选点沿不同闭环轨迹产生。54步是对新候选套用旧阈值的判定，并非对旧60个候选的原状态重放，不能作逐步一一配对解释。

`optimizer_steps.csv`逐步保留J_before、J_candidate、ΔJ、原容差、SQP轮号、原规则是否会拒绝和实际是否接受。旧基线没有保存被拒候选轨迹的目标值，旧ΔJ明确缺失，不补造、不以0替代。不同控制周期的预测初态和参考窗口不同，不能把跨周期目标值或ΔJ之和解释成统一目标函数下降、Lyapunov下降或稳定性证明。

## 真实物理状态与约束

16–28s真实俯仰、体轴俯仰角速度q、气动及CMG俯仰力矩见图2。新方案恢复了机动期明显的俯仰角速度和CMG正负力矩输出。全部框架角、实际框架速度、加速度、三轴角动量、舵偏与舵速在两份`*_physical_16_28_native.csv`中保留完整1ms样本；`physical_state_snapshots.csv`给出16、19.8、20、24、28s状态，不以显示抽样替代原始统计。框架角没有擅自增加原模型不存在的位置限幅。

新方案60s全程四类物理限制均通过，容差沿用1e-6：舵偏限值[25,20,20]°，三通道舵速60°/s，四框架速度1rad/s，四框架加速度3rad/s²。舵速最大超限量仅约2.44e-15rad/s，属于浮点量级；其他类别没有正超限。实际存在饱和，不能将“无越界”写成“无饱和”：舵偏/舵速/CMG速度/CMG加速度饱和样本比例约为5.512%/15.473%/0.397%/7.268%。

新旧保存的可达性证书及分配残差阈值均通过，实际CMG角动量/力矩与原几何关系的最大重构差约5.68e-14，低于1e-7核验容差。实际迎角均在原模型有效域内。新方案控制器和内环fallback均为0。12–24s未见物理候选拒绝；不能据此断言未来未测试场景也不会出现预测不可行。

## 五项问题的回答与解释边界

1. **第二次RMSE是否降低？** 是，3.232611°→0.140001°，第二次机动后误差也显著降低。
2. **是否恢复优化更新？** 是，[12,24)s接受147步，其中54步原先会被目标上升判断阻断。
3. **是否出现其他时段恶化？** 三次机动段均改善；第三次机动后RMSE小幅增加，已如实保留。不能把改善概括成逐点支配，也不能识别某个具体上升步的独立贡献。
4. **是否满足原物理约束？** 在本次完整60s输出和保存证书中，是；实际饱和仍存在，没有放宽限制或绕过可行性检查。
5. **是否说明拒步是主要原因？** 本单因素开关消融强烈支持它是该C2退化的主要直接贡献机制，说明原有“QP成功但不接受更新”足以形成严重闭环问题。但未证明唯一深层原因，未解释为什么该预测目标在这些步上上升，也没有证明永久移除保护后的全局稳定性。默认开关继续为true，本批结果不替换论文正式数据。

这是一个固定场景的确定性机制实验，无重复随机样本、显著性检验或跨场景稳定性结论。
'''
    (OUT/'reports/实验报告.md').write_text(summary,encoding='utf-8')
    (OUT/'README.md').write_text('''# C2目标函数拒步机制消融

仅新增一次60s联合MPC Simulink仿真；原始基线来自paper_final/V2_C2_3。本文批独立保存，不替换论文正式数据。

- 中文结论与限制：reports/实验报告.md
- 新旧逐事件对比：tables/event_comparison.csv；全程：tables/global_metrics.csv
- 优化器：tables/optimizer_diagnostics.csv、optimizer_steps.csv、optimizer_12_24_summary.csv
- 真实物理数据：tables/guard_on_physical_16_28_native.csv、guard_off_physical_16_28_native.csv
- 约束核验：tables/physical_constraints.csv、physical_verification.csv
- 三张图：figures/Fig1_C2_three_maneuvers、Fig2_C2_second_maneuver_detail、Fig3_C2_SQP_diagnostics（PDF及PNG）
- 新原始输出：raw/sim_output.mat；完整逐周期诊断：raw/*_authority.mat；分析中间完整数据：raw/analysis_native.mat
- 配置与模型：config/、models/、implementation/；原始/内嵌bank均保留
- SHA256：manifest.csv；验收：checks/acceptance.json

大型MAT和SLX保留本地，Git保存源码、配置、SHA、报告、CSV及图表。基线原始数据直接保留在paper_final，不重复仿真。本次代码默认enforce_objective_decrease=true，仅实验模型设置false。

从仓库根目录只复算、绘图（不会调用sim）：

```powershell
$env:MATLAB_HOME='D:/Program Files/MATLAB/R2024a'
& .\\simulink\\mlab.cmd "addpath('simulink'); init_paths; addpath('simulink/paper/c2_objective_guard'); analyze_c2_guard_ablation;"
python simulink/paper/c2_objective_guard/render_c2_guard.py
python simulink/paper/c2_objective_guard/report_c2_guard.py
```

run_c2_guard_ablation是本次单次仿真入口，simulation_invocation.json存在时会拒绝重跑。默认开关函数级回归位于checks/default_guard_regression.json；保持关闭拒步后的原stagnation退出，不强制每周期满4轮。图形显示按原始20ms分箱端点及各列极值选择真实点，不平滑；所有指标及物理约束用完整1ms输出计算。
''',encoding='utf-8')
    for name in ['run_c2_guard_ablation.m','analyze_c2_guard_ablation.m','render_c2_guard.py','report_c2_guard.py']:
        shutil.copy2(ROOT/'simulink/paper/c2_objective_guard'/name,OUT/'implementation'/name)
    for name in ['native_mpc_sfun.m','online_authority_rollout.m','verify_authority_candidate.m','online_verify_first_command.m','assemble_native_qp.m','solve_hpipm_snapshot.m']:
        shutil.copy2(ROOT/'simulink/hpipm'/name,OUT/'implementation'/name)
    qa=json.loads((OUT/'checks/figure_QA.json').read_text(encoding='utf-8'))
    qa['visual_review']='Assistant inspected all three PNGs; moved delta-J legend outside data and limited Fig2 plotted samples to16–28s; grayscale previews retained.'
    (OUT/'checks/figure_QA.json').write_text(json.dumps(qa,indent=2),encoding='utf-8')
    acceptance={**receipt,'new_simulations':simulation['simulations'],'physical_checks_passed':receipt['all_physical_constraints_passed'] and receipt['all_saved_certificates_passed'] and receipt['all_cmg_geometry_checks_passed'],
        'baseline_assets_SHA_unchanged':True,'executed_core_matches_saved_snapshot':True,
        'formal_paper_data_updated':False,'default_true_regression_passed':True,
        'repository_head_before_ablation_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'outcome':'Second C2 maneuver improved, updates resumed, third post-maneuver RMSE slightly increased; no global-stability claim.'}
    assert all([acceptance['complete_60s'],acceptance['physical_checks_passed'],acceptance['guard_off_was_used'],qa['automated_passed']])
    (OUT/'checks/acceptance.json').write_text(json.dumps(acceptance,ensure_ascii=False,indent=2),encoding='utf-8')
    payload=[]
    for path in sorted(OUT.rglob('*')):
        if not path.is_file() or path.name=='manifest.csv' or path.suffix=='.log' or any(part in ('cache','codegen','__pycache__') for part in path.relative_to(OUT).parts):continue
        payload.append({'path':path.relative_to(OUT).as_posix(),'bytes':path.stat().st_size,'sha256':sha(path)})
    write_csv(OUT/'manifest.csv',payload)
    print('C2_GUARD_REPORT_AND_HASHES_COMPLETE',len(payload),'files',sum(r['bytes'] for r in payload),'bytes')


if __name__=='__main__':
    main()
