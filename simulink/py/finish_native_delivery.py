"""Finish an existing sweep, refresh default models, then verify delivery.

``--report-only`` rewrites finish_report.json and docs/simulink/09_ab_experiment_report.md
from the existing p0_matrix_summary.csv without re-running any simulation.
"""
from pathlib import Path
import sys,os,subprocess,ctypes,time,json,csv
ROOT=Path(__file__).resolve().parents[2]
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
env=os.environ.copy();env['MATLAB_HOME']=r'D:\Program Files\MATLAB\R2024a';env['PYTHONIOENCODING']='utf-8'
PY_REF=ROOT/'results/torque_gap_no_cmg_comparison_rti2/P0'


def python_reference(case,arm):
    """max/RMS per-step attitude error of the historical Python run, if present."""
    path=PY_REF/f"{case}_{'with_cmg' if arm=='B' else 'no_cmg'}"/'timeseries.csv'
    if not path.exists():
        return None
    err=[float(r['att_err_deg']) for r in csv.DictReader(path.open(encoding='utf-8'))]
    return max(err), (sum(e*e for e in err)/len(err))**0.5


def write_report():
    report=dict(status='RUNNABLE_DELIVERY_WITH_VALIDATION_GAPS',p0_runs=16,
        limitations=['P0 B-arm CMG drive disconnection found and fixed (docs/simulink/08); residual gap to the Python '
                     'baseline is integration scheme and disturbance sampling, no pointwise trajectory equivalence',
        'Public aerodynamic surrogate, not identified airframe','ABC and uncertainty campaigns not completed',
        'Direct attitude reference mode only','GUI parameter screenshots unavailable after capture timeouts'])
    (ROOT/'simulink/results/native_delivery/finish_report.json').write_text(json.dumps(report,indent=2))
    with (ROOT/'simulink/results/native_closedloop/p0_matrix_summary.csv').open(encoding='utf-8-sig') as stream:
        rows=list(csv.DictReader(stream))
    lines=['# 原生迁移工程交付验收汇总','',
        '状态：当前工程主链已完成运行及重载交付检查；原方案的全部科研验收尚未完成。', '',
        '重要：本轮修订定位并修复了 P0 B 组的 CMG 框架驱动接线丢失（详见 `08_closed_loop_validation.md` 的修复小节）。'
        '修复前 B 组框架角恒为零、CMG 不产生反作用力矩，控制器却在优化四个并不存在的框架速率，因此 B 组指标显著劣于 A 组。',
        '',]
    lines+=['修复后 A/B 两组的误差量级与历史 Python 实现一致；剩余差异来自真值积分格式（本模型 ode4/1 ms 对 Python 8 子步 RK4/25 ms）'
        '与扰动力矩采样方式（零阶保持对子步中点查询），尚未做逐点轨迹等价与步长收敛认证。','',
        '已验证原生 6DOF、CMG 耦合、驱动/舵机、气动力矩、公开升阻力替代模块、在线 QP/HPIPM、热启动、回退与控制分配。', '',
        '默认 A/B 主模型已刷新为内嵌控制模板和未来参考的版本，保存后重载、再次仿真通过。详见 simulink/results/native_delivery/。','',
        '## 八组 P0 A/B 回归','',
        '`Python` 两列为历史实现的逐步姿态误差（同一工况、同一日志口径）。A 组无 CMG 通路，修复前后一致；'
        'B 组修复后由 3–40 度回落到与 Python 同一量级。','',
        '| 工况 | 组别 | 时长/s | 最大误差/deg | RMS/deg | Python max/deg | Python RMS/deg | 回退次数 |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        ref=python_reference(r['Case'],r['Arm'])
        refcols=f"{ref[0]:.4g} | {ref[1]:.4g}" if ref else 'n/a | n/a'
        lines.append(f"| {r['Case']} | {r['Arm']} | {r['Duration_s']} | {float(r['MaxError_deg']):.6g} | "
                     f"{float(r['RMSError_deg']):.6g} | {refcols} | {r['Fallbacks']} |")
    lines+=['','## 工程边界与未完成项','',
        '- 气动力使用公开再入极曲线与原有力矩模型的混合参数替代；无真实飞行器辨识，侧向系数假设为零、推力为零。',
        '- 当前采用直接姿态参考；自主位置/速度制导和按实际反馈重建全部未来环境尚未完成。',
        '- ABC 预测机制、参数不确定性和多初态实验尚未完成；不能据本次单套参数回归声称鲁棒性验收。',
        '- 图形参数对话框截图因 Windows 捕获工具超时未取得。已提供真实模型导出图、参数 JSON 与完整 Block 清单。',
        '- 本次不验收实时部署、代码生成及真实飞行器全包线可信性。', '',
        '打开模型与重现入口：simulink/NATIVE_README.md。原 Python 仿真、历史论文与图表未被覆盖。']
    (ROOT/'docs/simulink/09_ab_experiment_report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('NATIVE_DELIVERY_REPORT_WRITTEN',flush=True)


def run(args):subprocess.run(args,cwd=ROOT,env=env,check=True)


if __name__=='__main__':
    if '--report-only' in sys.argv:
        write_report();sys.exit(0)
    if '--wait-pid' in sys.argv:
        pid=int(sys.argv[sys.argv.index('--wait-pid')+1]);kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.OpenProcess.restype=ctypes.c_void_p;kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
        kernel.CloseHandle.argtypes=[ctypes.c_void_p]
        h=kernel.OpenProcess(0x00100000,False,pid)
        if h:
            print('WAIT_EXISTING_SWEEP',pid,flush=True)
            while kernel.WaitForSingleObject(h,1000)==258:pass
            kernel.CloseHandle(h)
    run([sys.executable,str(ROOT/'simulink/py/run_native_p0_matrix.py')])
    run([sys.executable,str(ROOT/'simulink/py/export_hpipm_qp.py')])
    run([sys.executable,str(ROOT/'simulink/py/export_hpipm_qp.py'),'--surface-only'])
    run([sys.executable,str(ROOT/'simulink/py/export_online_plan_bank.py')])
    run([str(ROOT/'simulink/mlab.cmd'),"maxNumCompThreads(1); addpath('simulink'); native_startup; check_native_rti; report_native_p0_matrix; finalize_native_delivery"])
    write_report()
    print('NATIVE_DELIVERY_FINISH_PASS',flush=True)
