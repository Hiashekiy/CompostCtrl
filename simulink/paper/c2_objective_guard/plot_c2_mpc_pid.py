"""C2 controller comparison from existing real Simulink MAT files; no simulation."""
import csv
import hashlib
import json
from pathlib import Path
import h5py
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'simulink/results/paper_experiments/C2_controller_comparison'
BASE=ROOT/'simulink/results/paper_final/E2_Controller_Comparison'
MPC=ROOT/'simulink/results/paper_experiments/C2_objective_guard_ablation'


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def saved_states(path):
    # These archived outputs have exactly one native10-state,60001-sample array.
    # Discover its actual HDF5 path, do not depend on MATLAB's reference numbering.
    with h5py.File(path) as f:
        times=[];states=[]
        def find(name,obj):
            if isinstance(obj,h5py.Dataset):
                if name.endswith('/tout') and obj.shape==(1,60001):times.append(name)
                if obj.shape==(10,60001) and obj.dtype.kind=='f':states.append(name)
        f.visititems(find)
        assert len(times)==len(states)==1,(times,states)
        t=f[times[0]][()].ravel();x=f[states[0]][()].T
    assert np.all(np.isfinite(x)) and np.max(np.abs(np.diff(t)-.001))<1e-8
    assert abs(t[-1]-60)<1e-9
    return t,x,{'time_dataset':times[0],'states_dataset':states[0]}


def reference(path,t):
    b=loadmat(path);bt=b['time'].ravel();q=b['q_ref'][:,0,:].T
    q=np.column_stack([np.interp(t,bt,q[:,j]) for j in range(4)])
    q/=np.linalg.norm(q,axis=1)[:,None]
    w,x,y,z=q.T
    pitch=np.rad2deg(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
    return q,pitch


def quaternion(e):
    cr,cp,cy=np.cos(e/2).T;sr,sp,sy=np.sin(e/2).T
    return np.column_stack([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                            cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy])


def extrema(values):
    n=len(values);m=n//20;indices=[np.arange(0,n,20),np.array([n-1])]
    for j in range(1,values.shape[1]):
        bins=values[:20*m,j].reshape(m,20)
        indices.extend([20*np.arange(m)+bins.argmin(axis=1),20*np.arange(m)+bins.argmax(axis=1)])
    return np.unique(np.concatenate(indices))


def main():
    for folder in ['figures','tables','reports']:(OUT/folder).mkdir(parents=True,exist_ok=True)
    pid_raw=BASE/'raw/V2_C2_1/sim_output.mat';mpc_raw=MPC/'raw/sim_output.mat'
    pid_bank=BASE/'scenarios/V2_C2_1/reference_bank.mat';mpc_bank=MPC/'config/reference_bank.mat'
    series={};provenance={};metrics=[];events=[]
    expected_pid=json.loads((BASE/'scenarios/V2_C2_1/metrics.json').read_text(encoding='utf-8'))['rmse_full_deg']
    with (MPC/'tables/global_metrics.csv').open(encoding='utf-8-sig',newline='') as f:
        expected_mpc=float(next(r for r in csv.DictReader(f) if r['method']=='guard_off')['rmse_full_60s_deg'])
    for method,raw,bank,expected in [('PID',pid_raw,pid_bank,expected_pid),('MPC',mpc_raw,mpc_bank,expected_mpc)]:
        t,x,paths=saved_states(raw);qr,ref=reference(bank,t);q=quaternion(x[:,:3])
        err=np.rad2deg(2*np.arccos(np.clip(np.abs(np.sum(q*qr,axis=1)),0,1)))
        rmse=float(np.sqrt(np.mean(err**2)));assert abs(rmse-expected)<1e-8,(method,rmse,expected)
        full=np.column_stack([t,np.rad2deg(x[:,1]),ref,err]);series[method]=full[extrema(full)]
        provenance[method]={'raw':raw.relative_to(ROOT).as_posix(),'raw_sha256':sha(raw),
            'reference_bank':bank.relative_to(ROOT).as_posix(),'reference_bank_sha256':sha(bank),
            **paths,'native_samples':len(t),'original_rmse_deg':expected,'recomputed_rmse_deg':rmse}
        metrics.append({'method':method,'rmse_full_deg':rmse,'peak_attitude_error_deg':float(err.max()),'native_samples':len(t)})
        for j,start in enumerate([8,20,32],1):
            mask=(t>=start)&(t<=start+4);stop=min(start+14,[20,32,60][j-1]);post=(t>start+4)&(t<=stop)
            events.append({'method':method,'event_index':j,'onset_s':start,'finish_s':start+4,
                'maneuver_rmse_deg':float(np.sqrt(np.mean(err[mask]**2))),
                'maneuver_peak_deg':float(err[mask].max()),'post_end_s':stop,
                'post_rmse_deg':float(np.sqrt(np.mean(err[post]**2)))})
        with (OUT/'tables'/f'{method}_plot_data.csv').open('w',encoding='utf-8',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','pitch_deg','ref_pitch_deg','attitude_error_deg']);w.writerows(series[method])
    assert np.array_equal(reference(pid_bank,t)[0],reference(mpc_bank,t)[0]),'References differ'
    for name,records in [('global_metrics.csv',metrics),('event_metrics.csv',events)]:
        with (OUT/'tables'/name).open('w',encoding='utf-8',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,
        'xtick.labelsize':8,'ytick.labelsize':8,'legend.fontsize':8,'pdf.fonttype':42,
        'path.simplify':False,'lines.linewidth':1.2,'savefig.dpi':300,
        'axes.spines.top':False,'axes.spines.right':False})
    fig,axs=plt.subplots(2,1,figsize=(7.2,4.8),layout='constrained')
    for method,color,style,label in [('MPC','#7B3294','-','MPC + CMG'),('PID','#D55E00','--','PID + CMG')]:
        d=series[method]
        axs[0].plot(d[:,0],d[:,1],color=color,ls=style,label=label)
        axs[1].plot(d[:,0],d[:,3],color=color,ls=style,label=label)
    ref=series['MPC'];axs[0].plot(ref[:,0],ref[:,2],'k:',lw=1,label='Pitch reference')
    for ax,y in zip(axs,['Pitch (deg)','Attitude error (deg)']):
        ax.set(xlabel='Time (s)',ylabel=y,xlim=(0,60));ax.grid(True,alpha=.25)
        for start in [8,20,32]:
            ax.axvspan(start,start+4,color='.5',alpha=.10)
            ax.axvline(start,color='.45',lw=.6,ls=':')
    for j,start in enumerate([8,20,32],1):
        axs[0].text(start+2,.97,f'M{j}',transform=axs[0].get_xaxis_transform(),ha='center',va='top',fontsize=8)
    handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside upper center',ncol=3)
    fig.canvas.draw();width,height=fig.canvas.get_width_height()
    for ax in axs:
        for text in [ax.xaxis.label,ax.yaxis.label]:
            box=text.get_window_extent(fig.canvas.get_renderer())
            assert box.x0>=0 and box.y0>=0 and box.x1<=width and box.y1<=height
    dest=OUT/'figures/C2_MPC_vs_PID'
    fig.savefig(dest.with_suffix('.png'));fig.savefig(dest.with_suffix('.pdf'),metadata={'CreationDate':None,'ModDate':None})
    plt.close(fig)
    profile={'methods':['MPC + CMG','PID + CMG'],'source':provenance,'new_simulations':0,
        'sampling':'Native1ms metrics;20ms bins retain endpoints and every-column extrema; no smoothing',
        'matched_reference':True,'native_metrics_match_saved':True,
        'note':'MPC uses latest completed C2 result; PID uses archived frozen PID, no unload; old guard-on MPC is not plotted.'}
    (OUT/'reports/provenance.json').write_text(json.dumps(profile,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'README.md').write_text('''# C2联合MPC与PID对比图

图中仅比较联合MPC、PID–CMG及俯仰参考，不再显示拒步开关对比。上图为实际俯仰，下图为四元数姿态角误差；灰色区间标记三次机动。

MPC读取C2_objective_guard_ablation中已完成的当前MPC原始输出；PID读取paper_final的V2_C2_1冻结PID无卸载原始输出。未新增仿真。两个原始bank参考一致，完整1ms重算RMSE与已保存指标差小于1e-8。绘图选择真实点，不平滑。

文件：figures/C2_MPC_vs_PID.png、figures/C2_MPC_vs_PID.pdf。原始数据路径与SHA256见reports/provenance.json，指标见tables/。

建议图注：C2连续俯仰机动中联合MPC与PID–CMG的跟踪响应及姿态误差对比（角度单位：°）。
''',encoding='utf-8')
    print('C2_MPC_PID_FIGURE_COMPLETE',dest.as_posix())
    print(json.dumps(metrics))


if __name__=='__main__':main()
