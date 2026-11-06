"""Unified static figures from archived MATLAB results; no physical modelling."""
import csv,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.text import Text
_HERE=Path(__file__).resolve();OUT=_HERE.parents[2] if (_HERE.parents[2]/'manifest.csv').exists() else _HERE.parents[3]/'simulink/results/paper_final';ROOT=OUT.parents[2]
G=['E1_CMG_Benefit','E2_Controller_Comparison','E3_Prediction_Horizon','E4_Frequency_Tracking']
C={'SURFACE_MPC':'#0072B2','PID_CMG':'#D55E00','PID_DYNAMIC_UNLOAD':'#009E73','JOINT_MPC':'#7B3294'}
LABEL={'SURFACE_MPC':'Surface MPC','PID_CMG':'Frozen PID + CMG','PID_DYNAMIC_UNLOAD':'PID dynamic unload','JOINT_MPC':'Joint MPC'}
STYLE={'SURFACE_MPC':'-','PID_CMG':'--','PID_DYNAMIC_UNLOAD':'-.','JOINT_MPC':':'}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,'axes.titlesize':9,'legend.fontsize':7,'xtick.labelsize':8,'ytick.labelsize':8,'pdf.fonttype':42,'ps.fonttype':42,'lines.linewidth':1.2,'axes.linewidth':.7,'savefig.dpi':300,'path.simplify':False,'axes.spines.top':False,'axes.spines.right':False,'figure.constrained_layout.use':True})
def rows(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def num(r,k):
 try:return float(r[k])
 except (ValueError,TypeError):return math.nan
def table(group,name):return rows(OUT/group/'tables'/name)
def trace(cid):
 r=rows(OUT/'.work'/f'{cid}.csv');return {k:np.array([num(x,k) for x in r]) for k in r[0]}
def line(ax,d,key,method,label=None):ax.plot(d['time_s'],d[key],color=C[method],ls=STYLE[method],label=label or LABEL[method])
def decorate(ax,ylabel,xlabel='Time (s)',legend=True):
 ax.set(xlabel=xlabel,ylabel=ylabel);ax.grid(True,alpha=.25)
 if legend:ax.legend(loc='best',framealpha=.9)
records=[];audits=[]
def save(fig,group,name,caption,case_ids):
 fig.canvas.draw();w,h=fig.canvas.get_width_height();bad=[]
 for ax in fig.axes:
  for text in [ax.title,ax.xaxis.label,ax.yaxis.label]:
   if text.get_visible() and text.get_text():
    b=text.get_window_extent(fig.canvas.get_renderer())
    if b.x0< -2 or b.y0< -2 or b.x1>w+2 or b.y1>h+2:bad.append(text.get_text())
 assert not bad,(name,bad)
 dest=OUT/group/'figures'/name;fig.savefig(dest.with_suffix('.png'));fig.savefig(dest.with_suffix('.pdf'),metadata={'CreationDate':None,'ModDate':None})
 audits.append({'figure':name,'axis_labels_inside_canvas':True,'font':'DejaVu Sans','font_pt':9,'width_in':fig.get_size_inches()[0],'height_in':fig.get_size_inches()[1],'DPI':300,'vector_PDF':True,'no_smoothing':True,'methods_have_fixed_colors_and_line_styles':True})
 records.append({'experiment':group,'figure':name,'pdf':dest.with_suffix('.pdf').relative_to(OUT).as_posix(),'png':dest.with_suffix('.png').relative_to(OUT).as_posix(),'caption':caption,'case_ids':';'.join(case_ids)})
 # Re-render the same plotted data in grayscale for accessibility QA (not image editing).
 from matplotlib.colors import to_rgb
 from matplotlib.lines import Line2D
 from matplotlib.collections import Collection
 for item in fig.findobj(match=Line2D):
  rgb=to_rgb(item.get_color());g=sum(a*b for a,b in zip(rgb,(.2126,.7152,.0722)));item.set_color((g,g,g))
 for ax in fig.axes:
  for item in ax.images:item.set_cmap('gray')
 for item in fig.findobj(match=Collection):
  if item.get_array() is not None:item.set_cmap('gray')
  else:
   for getter,setter in [(item.get_facecolors,item.set_facecolors),(item.get_edgecolors,item.set_edgecolors)]:
    rgba=getter()
    if len(rgba):g=rgba[:,:3]@np.array([.2126,.7152,.0722]);rgba[:,:3]=g[:,None];setter(rgba)
 qa=OUT/'reports/visual_QA';qa.mkdir(exist_ok=True);fig.savefig(qa/(name+'_gray.png'))
 plt.close(fig)
def event_marks(ax,on=35,end=40):
 ax.axvline(on,color='.4',lw=.7,ls='--');ax.axvline(end,color='.4',lw=.7,ls=':')
def main():
 manifest=rows(OUT/'manifest.csv');ids=[r['case_id'] for r in manifest]
 # Profile the fixed experimental design before choosing plots (no inferential error bars).
 profile={}
 for group in G:
  subset=[r for r in manifest if r['experiment']==group];profile[group]={'cases':len(subset),'methods':sorted(set(r['method'] for r in subset)),'status_counts':{s:sum(r['status']==s for r in subset) for s in sorted(set(r['status'] for r in subset))},'design':'deterministic cases, not independent repeated samples','chosen_plot':'matrix and ordered curves; actual time traces; no invented confidence intervals'}
 (OUT/'reports/data_profile.json').write_text(json.dumps(profile,indent=2),encoding='utf-8')
 e1=table(G[0],'E1_all.csv');amps=sorted({num(r,'amplitude_deg') for r in e1});dur=sorted({num(r,'transition_s') for r in e1});fig,axs=plt.subplots(1,2,figsize=(7.2,3.5));cmap=ListedColormap(['#E69F00','#D9D9D9','#009E73'])
 for ax,cmg in zip(axs,(0,1)):
  Z=np.full((len(amps),len(dur)),np.nan)
  for r in e1:
   if int(r['cmg'])==cmg:Z[amps.index(num(r,'amplitude_deg')),dur.index(num(r,'transition_s'))]=2 if r['status']=='SUCCESS' else 0 if r['status']=='MODEL_DOMAIN_FAIL' else 1
  im=ax.imshow(Z,origin='lower',aspect='auto',cmap=cmap,norm=BoundaryNorm([-.5,.5,1.5,2.5],3));ax.set(xticks=range(len(dur)),xticklabels=[f'{v:g}' for v in dur],yticks=range(len(amps)),yticklabels=[f'{v:g}' for v in amps],xlabel='Transition time (s)',ylabel='Pitch increment (deg)',title='Joint MPC' if cmg else 'Surface MPC')
 fig.colorbar(im,ax=axs,ticks=[0,1,2],shrink=.7).ax.set_yticklabels(['Model domain','Tracking fail','Success']);save(fig,G[0],'E1_feasibility_matrix','All144 formal cases; model-domain state separated from control failure.',[r['id'] for r in e1])
 fig,ax=plt.subplots(figsize=(7.2,3.2))
 for cmg,method in ((0,'SURFACE_MPC'),(1,'JOINT_MPC')):
  ys=[min([num(r,'transition_s') for r in e1 if int(r['cmg'])==cmg and num(r,'amplitude_deg')==a and r['status']=='SUCCESS'],default=math.nan) for a in amps]
  ax.plot(amps,ys,marker='o',color=C[method],ls=STYLE[method],label=LABEL[method])
 decorate(ax,'Minimum successful transition (s)','Pitch increment (deg)');save(fig,G[0],'E1_minimum_transition','Minimum over sampled transition durations; missing points mean no sampled successful case, not proof of physical impossibility.',[r['id'] for r in e1])
 for pair,label in [('V1_0050_0020','both_success'),('V1_0075_0020','joint_only'),('V1_0250_0200','high_alpha_range')]:
  fig,axs=plt.subplots(3,2,figsize=(7.2,6.8));a=trace(pair+'_A');b=trace(pair+'_B')
  for method,d in [('SURFACE_MPC',a),('JOINT_MPC',b)]:
   line(axs[0,0],d,'pitch_deg',method);line(axs[0,1],d,'attitude_error_deg',method)
   axs[1,0].plot(d['time_s'],np.rad2deg(d['elevator_rad']),color=C[method],ls=STYLE[method],label=LABEL[method]);line(axs[1,1],d,'aero_pitch_Nm',method)
  axs[0,0].plot(b['time_s'],b['ref_pitch_deg'],'k--',lw=.9,label='Reference');axs[1,0].axhline(25,color='.4',lw=.7,ls=':');axs[1,0].axhline(-25,color='.4',lw=.7,ls=':')
  line(axs[2,0],b,'cmg_pitch_Nm','JOINT_MPC');axs[2,1].plot(b['time_s'],np.max(abs(np.column_stack([b[f'nu{i}'] for i in range(1,5)])),axis=1),color=C['JOINT_MPC'],label='Max rate / limit');axs[2,1].plot(b['time_s'],np.max(abs(np.column_stack([b[f'acc{i}'] for i in range(1,5)])),axis=1)/3,color='.3',ls='--',label='Max acceleration / limit');axs[2,1].axhline(1,color='k',lw=.7,ls=':')
  for ax,y in zip(axs.flat,['Pitch (deg)','Attitude error (deg)','Elevator (deg)','Actual aero pitch (N m)','Actual CMG pitch (N m)','CMG limit utilisation']):decorate(ax,y);event_marks(ax,8,10 if '0020' in pair else 28)
  save(fig,G[0],'E1_representative_'+label,'Actual native responses; fixed original limits; range case is not labelled a model-domain failure.',[pair+'_A',pair+'_B'])
 # E2 single and continuous cases keep negative outcomes.
 fig,axs=plt.subplots(2,3,figsize=(7.2,4.8))
 for j,a in enumerate((5,10,15)):
  for m,method in ((1,'PID_CMG'),(3,'JOINT_MPC')):
   d=trace(f'V2_S{a:02d}_{m}');line(axs[0,j],d,'pitch_deg',method);line(axs[1,j],d,'attitude_error_deg',method)
  axs[0,j].plot(d['time_s'],d['ref_pitch_deg'],'k--',lw=.8,label='Reference');axs[0,j].set_title(f'{a} deg increment')
  for row,y in ((0,'Pitch (deg)'),(1,'Attitude error (deg)')):decorate(axs[row,j],y);event_marks(axs[row,j],8,13)
 handles,labels=axs[0,0].get_legend_handles_labels()
 for ax in axs.flat:ax.get_legend().remove()
 fig.legend(handles,labels,loc='outside upper center',ncol=3)
 save(fig,G[1],'E2_single_maneuvers','Frozen PID versus joint MPC; duplicate fixed-unload PID curves omitted.',[f'V2_S{a:02d}_{m}' for a in (5,10,15) for m in (1,3)])
 for c in (1,2):
  fig,axs=plt.subplots(3,1,figsize=(7.2,6.3))
  for m,method in ((1,'PID_CMG'),(3,'JOINT_MPC')):
   d=trace(f'V2_C{c}_{m}');line(axs[0],d,'pitch_deg',method);line(axs[1],d,'roll_deg' if c==1 else 'attitude_error_deg',method)
  axs[0].plot(d['time_s'],d['ref_pitch_deg'],'k--',lw=.8,label='Reference')
  if c==1:
   axs[1].plot(d['time_s'],d['ref_roll_deg'],'k--',lw=.8,label='Reference')
   for m,method in ((1,'PID_CMG'),(3,'JOINT_MPC')):line(axs[2],trace(f'V2_C{c}_{m}'),'attitude_error_deg',method)
   for ax in axs:ax.axvspan(52,60,color='.5',alpha=.12)
  else:
   S=table(G[1],'C2_solver_diagnostics.csv');tt=np.array([num(r,'time_s') for r in S]);axs[2].step(tt,[num(r,'accepted_steps') for r in S],where='post',color=C['JOINT_MPC'],label='Accepted steps');axs[2].step(tt,[num(r,'rejected_steps') for r in S],where='post',color='.3',ls='--',label='Rejected steps')
   for ax in axs:ax.axvspan(12,24,color='.5',alpha=.12)
  ys=['Pitch (deg)','Roll (deg)' if c==1 else 'Attitude error (deg)','Attitude error (deg)' if c==1 else 'MPC step counts']
  for ax,y in zip(axs,ys):decorate(ax,y)
  save(fig,G[1],f'E2_C{c}_continuous','C1 final post-window is8s.' if c==1 else 'C2 second-maneuver degradation retained; successful QPs with rejected full steps do not identify the unique deeper cause.',[f'V2_C{c}_{m}' for m in (1,3)])
 fig,axs=plt.subplots(2,2,figsize=(7.2,5.0));F=table(G[1],'M1M2_offline_feasibility.csv')
 for j in range(2):
  Q=[r for r in F if int(r['task'])==j+1];t=[num(r,'time_s') for r in Q];axs[0,j].plot(t,[num(r,'qbar_Pa') for r in Q],color='k');axs[1,j].plot(t,[num(r,'feasible_extra_norm_Nm') for r in Q],color=C['PID_DYNAMIC_UNLOAD']);axs[0,j].set_title(f'M{j+1}: A2-12; B12-35; C35-40 s')
  for ax,y in ((axs[0,j],'Dynamic pressure (Pa)'),(axs[1,j],'Coupled feasible extra moment (N m)')):decorate(ax,y,legend=False);event_marks(ax);ax.axvline(12,color='.5',lw=.7,ls=':')
 save(fig,G[1],'E2_unload_authority_window','Offline conditional feasibility on original physical flight; not a time-based force switch.',[f'E2R_M{j}_{m}' for j in (1,2) for m in (1,2,3)])
 fig,axs=plt.subplots(3,2,figsize=(7.2,7.0));fig2,bxs=plt.subplots(2,2,figsize=(7.2,5.0));fig3,cxs=plt.subplots(2,2,figsize=(7.2,5.0));fig4,gxs=plt.subplots(1,2,figsize=(7.2,3.2))
 for j in range(2):
  for m,method in ((1,'PID_CMG'),(2,'PID_DYNAMIC_UNLOAD'),(3,'JOINT_MPC')):
   d=trace(f'E2R_M{j+1}_{m}');h=np.sqrt(sum(d[k]**2 for k in ('Hx','Hy','Hz')));axs[0,j].plot(d['time_s'],h,color=C[method],ls=STYLE[method],label=LABEL[method]);line(axs[1,j],d,'sigma_min',method);line(axs[2,j],d,'positive_support_Nm',method);line(bxs[0,j],d,'pitch_deg',method);line(bxs[1,j],d,'attitude_error_deg',method)
  bxs[0,j].plot(d['time_s'],d['ref_pitch_deg'],'k--',lw=.8,label='Reference');axs[0,j].set_title(f'M{j+1}');bxs[0,j].set_title(f'M{j+1}')
  for m,method in ((1,'PID_CMG'),(2,'PID_DYNAMIC_UNLOAD')):
   d=trace(f'E2R_M{j+1}_{m}');cxs[0,j].plot(d['time_s'],d['aero_pitch_Nm'],color=C[method],label=LABEL[method]+' aero');cxs[0,j].plot(d['time_s'],d['cmg_pitch_Nm'],color=C[method],ls='--',label=LABEL[method]+' CMG')
  L=table(G[1],f'E2R_M{j+1}_2_dynamic_unload_ticks.csv');tt=[num(r,'time_s') for r in L]
  for key,label,col,ls in [('actual_extra_aero_y_Nm','Measured conditional aero extra',C['PID_CMG'],'-'),('actual_extra_cmg_y_Nm','Measured conditional CMG extra',C['PID_DYNAMIC_UNLOAD'],'--'),('actual_compensation_error_Nm','3-axis compensation error norm','.3',':')]:cxs[1,j].plot(tt,[num(r,key) for r in L],label=label,color=col,ls=ls)
  cxs[1,j].axhline(.25,color='k',ls=':',lw=.7,label='Original tolerance');cxs[0,j].set_title(f'M{j+1}')
  pair=rows(OUT/'.work'/f'E2_config_pair_{j+1}.csv');pt=[num(r,'time_s') for r in pair]
  for frame in range(1,5):gxs[j].plot(pt,[num(r,f'dg{frame}_deg') for r in pair],label=f'Frame{frame}',ls=['-','--','-.',':'][frame-1])
  gxs[j].set_title(f'M{j+1}: dynamic minus no unload');decorate(gxs[j],'Gimbal difference (deg)');event_marks(gxs[j])
  for i,y in enumerate(['H norm (N m s)','Minimum singular value','Positive pitch projection (N m)']):decorate(axs[i,j],y);event_marks(axs[i,j])
  for i,y in enumerate(['Pitch (deg)','Attitude error (deg)']):decorate(bxs[i,j],y);event_marks(bxs[i,j]);bxs[i,j].set_xlim(30,50)
  for i,y in enumerate(['Actual pitch moments (N m)','Conditional endpoint increment (N m)']):decorate(cxs[i,j],y);event_marks(cxs[i,j]);cxs[i,j].set_xlim(0,40)
 all_unload=[f'E2R_M{j}_{m}' for j in (1,2) for m in (1,2,3)]
 handles,labels=axs[0,0].get_legend_handles_labels()
 for ax in axs.flat:ax.get_legend().remove()
 fig.legend(handles,labels,loc='outside upper center',ncol=3)
 handles,labels=bxs[0,0].get_legend_handles_labels()
 for ax in bxs.flat:ax.get_legend().remove()
 fig2.legend(handles,labels,loc='outside upper center',ncol=2)
 ha,la=cxs[0,0].get_legend_handles_labels();hb,lb=cxs[1,0].get_legend_handles_labels()
 for ax in cxs.flat:ax.get_legend().remove()
 fig3.legend(ha+hb,la+lb,loc='outside upper center',ncol=2)
 for f,name,cap in [(fig,'E2_unload_state','H norm reduction does not imply universal directional authority or tracking improvement.'),(fig2,'E2_unload_following_maneuver','M1 modest improvement and M2 deterioration are both retained.'),(fig3,'E2_unload_measured_compensation','Conditional no-unload endpoint comparison; not a second counterfactual flight.'),(fig4,'E2_unload_gimbal_change','Four actual gimbal-state changes relative to the no-unload trajectory.')]:save(f,G[1],name,cap,all_unload)
 # E3 ordered horizon curves and complete five paired migration traces.
 R=table(G[2],'E3_10cases.csv');M=sorted([r for r in R if r['task']=='M'],key=lambda r:num(r,'horizon_s'));h=np.array([num(r,'horizon_s') for r in M]);fig,axs=plt.subplots(2,2,figsize=(7.2,5.2))
 for key,label,mark in [('maneuver_rmse_deg','35-40s','o'),('post_rmse_deg','40-50s','s'),('rmse_full_deg','Full60s','^')]:axs[0,0].plot(h,[num(r,key) for r in M],marker=mark,label=label)
 for key,label,mark in [('mean_compute_s','Mean','o'),('p95_compute_s','P95','s'),('max_compute_s','Maximum','^')]:axs[0,1].plot(h,[num(r,key) for r in M],marker=mark,label=label)
 P=table(G[2],'pairs_from_raw.csv');axs[1,0].plot(h,[num(r,'advance_s') for r in P],'o-',color=C['JOINT_MPC'],label='2deg sustained0.6s');axs[1,1].plot(h,[num(r,'over200ms_pct') for r in M],'o-',color=C['JOINT_MPC'])
 for ax,y in zip(axs.flat,['Attitude RMSE (deg)','Solve time (s)','Task-relative migration advance (s)','Solve time >200ms (%)']):decorate(ax,y,'Horizon (s)',legend=ax is not axs[1,1])
 save(fig,G[2],'E3_horizon_performance','Observed association, not unique causation; reused/new solve times have different run loads.',[r['id'] for r in R])
 fig,axs=plt.subplots(5,2,figsize=(7.2,9))
 for j,H in enumerate(h):
  D=rows(OUT/'.work'/f'E3_pair_{H:g}.csv');t=[num(r,'time_s') for r in D];axs[j,0].plot(t,[num(r,'max_dgamma_deg') for r in D],color=C['JOINT_MPC']);axs[j,0].axhline(2,color='.3',ls=':',lw=.7);axs[j,1].plot(t,[num(r,'dHy') for r in D],color=C['JOINT_MPC'])
  axs[j,0].set_title(f'H={H:g} s')
  for ax,y in ((axs[j,0],'Gimbal diff. (deg)'),(axs[j,1],'Delta Hy (N m s)')):decorate(ax,y,legend=False);ax.axvline(35-H,color='.5',ls=':',lw=.7);ax.axvline(35,color='k',ls='--',lw=.7);ax.set_xlim(0,35)
 save(fig,G[2],'E3_task_relative_migration','Each horizon uses its own M/Z simulation; dotted preview onset and dashed35s task onset.',[r['id'] for r in R])
 P=table(G[2],'E3_5pairs.csv');fig,axs=plt.subplots(2,2,figsize=(7.2,5.0))
 for ax,key,y in zip(axs.flat,['Hy_at_onset_Nms','sigma_min_at_onset','pitch_positive_support_at_onset_Nm','pitch_negative_support_at_onset_Nm'],['Hy at34.8s (N m s)','Minimum singular value at34.8s','Positive pitch projection (N m)','Negative pitch projection (N m)']):
  for tag,col,ls in [('M',C['JOINT_MPC'],'-'),('Z','.35','--')]:ax.plot(h,[num(r,tag+'_'+key) for r in P],color=col,ls=ls,marker='o',label=tag)
  decorate(ax,y,'Horizon (s)')
 save(fig,G[2],'E3_local_directional_authority','Signed, rate/acceleration-constrained projection bounds; not isolated pure pitch torque.',[r['id'] for r in R])
 # Exactly three core E4 figures, all27 data points retained.
 E=table(G[3],'E4_1deg_27cases.csv');mapping={'SURFACE_MPC':'SURFACE_MPC','PID_CHAIN_CMG':'PID_CMG','JOINT_MPC_CMG':'JOINT_MPC'}
 for key,name,y in [('gain_db','E4_gain','Amplitude gain (dB)'),('phase_lag_deg','E4_phase','Phase lag (deg)'),('pitch_rmse_deg','E4_rmse','Pitch tracking RMSE (deg)')]:
  fig,ax=plt.subplots(figsize=(7.2,3.4))
  for source,method in mapping.items():
   q=sorted([r for r in E if r['display_method']==source],key=lambda r:num(r,'frequency_Hz'));x=np.array([num(r,'frequency_Hz') for r in q]);ydata=np.array([num(r,key) for r in q]);unreliable=np.array([r['phase_unreliable']=='1' for r in q])
   if key=='phase_lag_deg':
    reliable=ydata.copy();reliable[unreliable]=np.nan;ax.plot(x,reliable,marker='o',ls=STYLE[method],color=C[method],label=LABEL[method])
    if np.any(unreliable):ax.scatter(x[unreliable],ydata[unreliable],marker='x',s=40,color=C[method],label=LABEL[method]+' unreliable')
   else:ax.plot(x,ydata,marker='o',ls=STYLE[method],color=C[method],label=LABEL[method])
  decorate(ax,y,'Frequency (Hz)');save(fig,G[3],name,'1degree peak sine; last2cycles; state varies with frequency/run duration; not strict fixed-point LTI bandwidth.',[r['id'] for r in E])
 with (OUT/'reports/figure_registry.csv').open('w',encoding='utf-8',newline='') as f:w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
 (OUT/'reports/figure_QA.json').write_text(json.dumps({'automated_passed':True,'figures':audits,'visual_review':'Representative assistant visual review is recorded separately in reports/visual_review.md; this automated run is not human review.'},indent=2),encoding='utf-8')
 print('FIGURES_COMPLETE',len(records))
if __name__=='__main__':main()
