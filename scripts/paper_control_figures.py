"""Compact paper figures, showing original samples up to 140 seconds."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BLUE='#0072B2'; ORANGE='#D55E00'; GREEN='#009E73'; GRAY='#56616d'
NAMES={'S00_pitch30':'正向30°俯仰','S01_pitch_minus30':'反向30°俯仰',
 'S02_pitch10':'10°俯仰','S03_pitch45':'45°俯仰','S04_roll30':'30°滚转',
 'S05_yaw30':'30°偏航','S06_pitch30_roll20':'俯仰–滚转耦合','S07_sequential':'连续机动'}

def layout(title,subtitle):
    plt.rcParams.update({'font.family':'Microsoft YaHei','font.size':10,
      'axes.labelsize':10,'axes.titlesize':11,'legend.fontsize':8,
      'axes.unicode_minus':False,'mathtext.fontset':'stix','pdf.fonttype':42,
      'svg.fonttype':'none','savefig.bbox':'tight'})
    fig,axs=plt.subplots(2,2,figsize=(11.2,6.6))
    fig.subplots_adjust(left=.075,right=.98,bottom=.10,top=.84,wspace=.25,hspace=.48)
    fig.suptitle(title,x=.075,y=.98,ha='left',fontsize=15,fontweight='bold')
    fig.text(.075,.915,subtitle,fontsize=9,color=GRAY)
    for ax in axs.flat:
        ax.spines[['top','right']].set_visible(False)
        ax.spines[['left','bottom']].set_color('#acb5bf')
        ax.grid(axis='y',color='#e1e6eb',lw=.6)
        ax.tick_params(length=3,color='#acb5bf')
        ax.set_xlabel('时间 / s')
        ax.margins(x=0,y=.16)
    return fig,axs.flat

def panel(ax,title,ylabel,t,window=None):
    ax.set_title(title,loc='left',pad=10,fontweight='medium')
    ax.set_ylabel(ylabel)
    ax.set_xlim(90,140)
    if window:
        ax.axvspan(window[0],window[1],color=BLUE,alpha=.065,lw=0,zorder=0)

def legend(ax,ncol=2):
    ax.legend(loc='best',ncol=ncol,frameon=True,facecolor='white',edgecolor='none',
      framealpha=.88,handlelength=2,columnspacing=1)

def save(fig,png):
    png=Path(png);png.parent.mkdir(parents=True,exist_ok=True)
    # Write a new file before replacing previews that may be open in an editor.
    temp=png.with_name(png.stem+'.render.png')
    fig.savefig(temp,dpi=220,pad_inches=.12)
    temp.replace(png)
    pdf=png.with_suffix('.pdf');temp=png.with_name(png.stem+'.render.pdf')
    fig.savefig(temp,pad_inches=.12)
    temp.replace(pdf)
    plt.close(fig)

def scenario(phase,sc,a,b,out_png,euler):
    n=min(len(a['t_next']),len(b['t_next']))
    n=min(n,int(np.count_nonzero(b['t_next'][:n]<=140+1e-8)))
    t=b['t_next'][:n]
    m=b['_metrics'];window=m['windows']['combined'];win=(window['t0'],window['t1'])
    axis=m['windows']['man0']['axis'];k={'roll':0,'pitch':1,'yaw':2}.get(axis,1)
    sym='xyz'[k];axisname=['滚转','俯仰','偏航'][k]
    q=lambda run,prefix:euler(np.column_stack([run[prefix+str(i)][:n] for i in range(4)]))[:,k]
    fig,axes=layout(f'{phase}  {NAMES.get(sc,sc)}：舵面与CMG协同控制',
      '显示90–140 s；浅蓝背景：机动窗口；浅绿区域及绿虚线：气动力矩可行区间与上下限')
    ax0,ax1,ax2,ax3=axes
    panel(ax0,f'(a) {axisname}姿态跟踪','姿态角 / °',t,win)
    ax0.plot(t,q(b,'qref'),color=GRAY,ls='--',lw=1.4,label='参考')
    ax0.plot(t,q(a,'q'),color=ORANGE,lw=1.6,label='仅舵面')
    ax0.plot(t,q(b,'q'),color=BLUE,lw=1.7,label='舵面+CMG');legend(ax0,3)
    mask=(t>=win[0])&(t<min(win[1],140+1e-8))
    ra=float(np.sqrt(np.mean(a['att_err_deg'][:n][mask]**2)))
    rb=float(np.sqrt(np.mean(b['att_err_deg'][:n][mask]**2)))
    panel(ax1,'(b) 三轴姿态误差范数','误差 / °',t,win)
    ax1.plot(t,a['att_err_deg'][:n],color=ORANGE,lw=1.6,label=f'仅舵面  RMSE {ra:.3f}°')
    ax1.plot(t,b['att_err_deg'][:n],color=BLUE,lw=1.7,label=f'舵面+CMG  RMSE {rb:.3f}°')
    legend(ax1,1)
    ax1.text(.02,.02,'RMSE：图内机动窗口',transform=ax1.transAxes,fontsize=7.5,color=GRAY)
    panel(ax2,f'(c) 联合控制的{axisname}轴力矩分工','力矩 / N·m',t,win)
    # Input/averaged torque diagnostics belong to interval start t, not t_next.
    tc=b['t'][:n]
    authority(ax2,tc,b['lo_'+sym][:n],b['hi_'+sym][:n])
    ax2.plot(tc,b['tau_act_'+sym][:n],color=GREEN,lw=1.5,label='气动总力矩')
    ax2.plot(tc,b['cmg_'+sym][:n],color=BLUE,lw=1.6,label='CMG力矩')
    ax2.plot(tc,b['tau_nom_'+sym][:n],color=GRAY,ls='--',lw=1.1,label='参考需求')
    legend(ax2,3)
    panel(ax3,f'(d) CMG{axisname}轴角动量','角动量 / N·m·s',t,win)
    ax3.plot(t,b['H'+sym][:n],color=BLUE,lw=1.7)
    ax3.axhline(0,color='#aeb7c1',lw=.7)
    ax3.text(.03,.95,'CMG内部状态；与瞬时输出力矩区分',transform=ax3.transAxes,
      va='top',fontsize=8,color=GRAY,bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
    save(fig,out_png)

def mechanism(g,st,out_png):
    raw=g['d'];full_t=raw.get('t_next',raw['t']+.2)
    mask=full_t<=140+1e-8
    d={k:(v[mask] if isinstance(v,np.ndarray) and len(v)==len(mask) else v) for k,v in raw.items()}
    t=d.get('t_next',d['t']+.2);tag=g['tag']
    name={'A':'L 完整时域 23.2 s','B':'S 短时域 5.0 s','C':'Z 零机动对照'}[tag]
    onset=st['migration_onset_t']
    onsetnote=f'10°阈值迁移起点 {onset:.1f} s' if onset is not None else '未出现超过10°的提前迁移'
    fig,axes=layout('预测机制验证：'+name,onsetnote+'；显示90–140 s；浅蓝背景为110–130 s'+('观察窗口（无机动）' if tag=='C' else '机动窗口'))
    a,b,c,e=axes;win=(110,130)
    panel(a,'(a) 姿态误差','误差 / °',t,win)
    a.plot(t,d['att_err_deg'],color=BLUE,lw=1.7)
    gamma=np.column_stack([d['g'+str(i)] for i in range(1,5)])
    deviation=np.rad2deg(np.max(np.abs(gamma-gamma[0]),axis=1))
    panel(b,'(b) 构型相对初始状态的偏离','最大框架角偏移 / °',t,win)
    b.plot(t,deviation,color=BLUE,lw=1.7,label=r'$\max_i|\gamma_i-\gamma_{i,0}|$')
    b.axhline(10,color=GRAY,ls=':',lw=1,label='迁移判据 10°');legend(b,1)
    panel(c,'(c) 俯仰轴角动量预置与释放','角动量 / N·m·s',t,win)
    c.plot(t,d['Hy'],color=BLUE,lw=1.7)
    c.axhline(0,color='#acb5bf',lw=.7)
    if onset is not None:
        b.axvline(onset,color=ORANGE,ls='--',lw=1)
    panel(e,'(d) 俯仰轴力矩分工','力矩 / N·m',t,win)
    authority(e,d['t'],d['lo_y'],d['hi_y'])
    e.plot(d['t'],d['tau_act_y'],color=GREEN,lw=1.5,label='气动总力矩')
    e.plot(d['t'],d['cmg_y'],color=BLUE,lw=1.5,label='CMG力矩')
    e.plot(d['t'],d['tau_nom_y'],color=GRAY,ls='--',lw=1.1,label='参考需求');legend(e,3)
    save(fig,out_png)
    return Path(out_png),Path(out_png).with_suffix('.pdf')


def authority(ax,t,lo,hi):
    """Bounds are the same B-group predicted aerodynamic torque box as the MPC."""
    ax.fill_between(t,lo,hi,color=GREEN,alpha=.075,lw=0,zorder=0)
    ax.plot(t,hi,color=GREEN,ls='--',lw=.9,alpha=.8,label='气动上限')
    ax.plot(t,lo,color=GREEN,ls=':',lw=1.1,alpha=.8,label='气动下限')
