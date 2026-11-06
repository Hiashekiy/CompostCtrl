"""Three mechanism-ablation figures from MATLAB's native-data exports only."""
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.collections import Collection

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'simulink/results/paper_experiments/C2_objective_guard_ablation'
TABLES = OUT / 'tables'
COLORS = {'guard_on': '#0072B2', 'guard_off': '#D55E00'}
LABELS = {'guard_on': 'Original MPC: guard on', 'guard_off': 'Ablation MPC: guard off'}
STYLES = {'guard_on': '--', 'guard_off': '-'}
plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':9, 'axes.labelsize':9,
    'axes.titlesize':9, 'legend.fontsize':7, 'xtick.labelsize':8, 'ytick.labelsize':8,
    'pdf.fonttype':42, 'lines.linewidth':1.2, 'savefig.dpi':300, 'path.simplify':False,
    'axes.spines.top':False, 'axes.spines.right':False, 'figure.constrained_layout.use':True})


def rows(name):
    with (TABLES / name).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def number(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return math.nan


def arrays(records):
    return {key:np.array([number(row[key]) for row in records]) for key in records[0]}


def axis(ax, ylabel, limits=(0,60)):
    ax.set(xlabel='Time (s)', ylabel=ylabel, xlim=limits)
    ax.grid(True, alpha=.25)


def windows(ax):
    for start,end in [(8,12),(20,24),(32,36)]:
        ax.axvspan(start,end,color='.5',alpha=.10)
        ax.axvline(start,color='.45',lw=.6,ls=':')


audits=[]


def save(fig,name):
    fig.canvas.draw()
    width,height=fig.canvas.get_width_height()
    for ax in fig.axes:
        xticks=[tick.label1 for tick in ax.xaxis.get_major_ticks() if ax.get_xlim()[0]-1e-9<=tick.get_loc()<=ax.get_xlim()[1]+1e-9]
        yticks=[tick.label1 for tick in ax.yaxis.get_major_ticks() if ax.get_ylim()[0]-1e-9<=tick.get_loc()<=ax.get_ylim()[1]+1e-9]
        for text in [ax.title, ax.xaxis.label, ax.yaxis.label, *xticks, *yticks]:
            if text.get_visible() and text.get_text():
                b=text.get_window_extent(fig.canvas.get_renderer())
                assert b.x0>=-2 and b.y0>=-2 and b.x1<=width+2 and b.y1<=height+2,(name,text.get_text())
    dest=OUT/'figures'/name
    fig.savefig(dest.with_suffix('.png'))
    fig.savefig(dest.with_suffix('.pdf'),metadata={'CreationDate':None,'ModDate':None})
    # Independent accessibility preview, generated from the plotted data.
    for obj in fig.findobj(match=Line2D):
        from matplotlib.colors import to_rgb
        rgb=to_rgb(obj.get_color());g=np.dot(rgb,[.2126,.7152,.0722]);obj.set_color((g,g,g))
    for obj in fig.findobj(match=Collection):
        for getter,setter in [(obj.get_facecolors,obj.set_facecolors),(obj.get_edgecolors,obj.set_edgecolors)]:
            rgba=getter()
            if len(rgba):
                rgba[:,:3]=(rgba[:,:3]@np.array([.2126,.7152,.0722]))[:,None];setter(rgba)
    fig.savefig(OUT/'checks'/(name+'_grayscale.png'))
    audits.append({'name':name,'axis_text_inside_canvas':True,'font_pt':9,'PNG_dpi':300,
                   'vector_PDF':True,'no_smoothing':True,'width_in':float(fig.get_size_inches()[0])})
    plt.close(fig)


def main():
    data={method:arrays(rows(method+'_plot_timeseries.csv')) for method in COLORS}
    cycles=rows('optimizer_diagnostics.csv');steps=rows('optimizer_steps.csv')
    profile={'cases':2,'new_simulations':1,'baseline':'paper_final/V2_C2_3',
        'design':'one deterministic mechanism ablation, not independent repeated trials',
        'plot_points':{m:len(d['time_s']) for m,d in data.items()},
        'cycle_records':len(cycles),'candidate_step_records':len(steps),
        'baseline_candidate_deltas_available':False,
        'plots':'time curves, per-cycle steps and per-candidate scatter; no smoothing or inferential error bars'}
    (OUT/'checks/data_profile.json').write_text(json.dumps(profile,indent=2),encoding='utf-8')
    fig,axs=plt.subplots(2,1,figsize=(7.2,4.8))
    for method,d in data.items():
        for ax,key in zip(axs,['pitch_deg','attitude_error_deg']):
            ax.plot(d['time_s'],d[key],color=COLORS[method],ls=STYLES[method],label=LABELS[method])
    d=data['guard_on'];axs[0].plot(d['time_s'],d['ref_pitch_deg'],'k:',lw=1,label='Pitch reference')
    for ax,y in zip(axs,['Pitch (deg)','Attitude error (deg)']):
        windows(ax);axis(ax,y)
    for j,start in enumerate([8,20,32],1):
        axs[0].text(start+2,.97,f'M{j}',ha='center',va='top',transform=axs[0].get_xaxis_transform(),fontsize=8)
    handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside upper center',ncol=3)
    save(fig,'Fig1_C2_three_maneuvers')

    fig,axs=plt.subplots(4,1,figsize=(7.2,7.2))
    for method,d in data.items():
        local=(d['time_s']>=16)&(d['time_s']<=28)
        for ax,key in zip(axs,['pitch_deg','q_degps','aero_y_Nm','cmg_y_Nm']):
            ax.plot(d['time_s'][local],d[key][local],color=COLORS[method],ls=STYLES[method],label=LABELS[method])
    d=data['guard_on'];local=(d['time_s']>=16)&(d['time_s']<=28);axs[0].plot(d['time_s'][local],d['ref_pitch_deg'][local],'k:',lw=1,label='Pitch reference')
    for ax,y in zip(axs,['Pitch (deg)','Body pitch rate q (deg/s)','Actual aero pitch (N m)','Actual CMG pitch (N m)']):
        windows(ax);axis(ax,y,(16,28))
    handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside upper center',ncol=3)
    save(fig,'Fig2_C2_second_maneuver_detail')

    fig,axs=plt.subplots(4,1,figsize=(7.2,8.2))
    for method in COLORS:
        R=arrays([r for r in cycles if r['method']==method]);time=R['time_s']
        axs[0].step(time,R['accepted_steps'],where='post',color=COLORS[method],ls=STYLES[method],label=LABELS[method])
        axs[2].step(time,R['last_qp_status'],where='post',color=COLORS[method],ls=STYLES[method],label=LABELS[method],marker='o' if method=='guard_on' else 'x',markevery=10,ms=3)
        axs[3].step(time,R['rejection_code'],where='post',color=COLORS[method],ls=STYLES[method],label=LABELS[method])
    S=arrays(steps);inc=(S['would_objective_reject']>0)&(S['accepted']>0)
    axs[1].scatter(S['time_s'][~inc],S['delta_J'][~inc],s=7,facecolors='none',edgecolors=COLORS['guard_off'],linewidths=.5,label='Other feasible candidates')
    axs[1].scatter(S['time_s'][inc],S['delta_J'][inc],s=12,color=COLORS['guard_off'],marker='x',label='Accepted increase above tolerance')
    axs[1].axhline(0,color='.4',lw=.6);axs[1].set_yscale('symlog',linthresh=1)
    delta_handles,delta_labels=axs[1].get_legend_handles_labels()
    fig.legend(delta_handles,delta_labels,loc='outside lower center',ncol=2)
    axs[1].set_title('Candidate deltas recorded only for the guard-off run',fontsize=8)
    axs[0].set_yticks([0,1,2,3,4]);axs[2].set_ylim(-1.2,1.2);axs[2].set_yticks([-1,0,1])
    axs[3].set(yticks=[0,1,2,3,4],yticklabels=['None','Objective','Physical','QP failure','Fallback'],ylim=(-.25,4.25))
    for ax,y in zip(axs,['Accepted SQP steps / cycle','Candidate delta J (symlog)','Last QP status (0 = solved)','Cycle rejection / failure reason']):
        axis(ax,y,(12,36));windows(ax)
    handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside upper center',ncol=2)
    save(fig,'Fig3_C2_SQP_diagnostics')
    (OUT/'checks/figure_QA.json').write_text(json.dumps({'automated_passed':True,'figures':audits,'visual_review':'PNG previews must be inspected before delivery'},indent=2),encoding='utf-8')
    print('C2_GUARD_FIGURES_COMPLETE',len(audits))


if __name__=='__main__':
    main()
