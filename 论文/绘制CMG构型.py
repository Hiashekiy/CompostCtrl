"""按nearspace.cmg的几何定义绘制方向示意；安装位置无物理尺寸含义。"""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Arc
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from nearspace.cmg import CMGCluster,CMGParams

plt.rcParams.update({'font.family':'Microsoft YaHei','mathtext.fontset':'stix',
    'font.size':11,'axes.unicode_minus':False,'svg.fonttype':'none','pdf.fonttype':42})
out=ROOT/'论文/figures';out.mkdir(exist_ok=True)
cl=CMGCluster(CMGParams(h0=50))
g,s=cl.g_axis,cl.s0
assert np.allclose(np.sum(g*s,axis=0),0)
assert np.allclose(np.linalg.norm(g,axis=0),1)
colors=['#0072B2','#D55E00','#009E73','#9B5798']
fig=plt.figure(figsize=(13.4,6.4),facecolor='white')
ax=fig.add_axes([.015,.12,.53,.79],projection='3d',computed_zorder=False)
ax.set_proj_type('ortho'); ax.view_init(elev=23,azim=38)
centers=1.45*g.T
ax.add_collection3d(Poly3DCollection([centers],facecolor='#e9edf1',edgecolor='#aeb7c2',alpha=.24,lw=1))
for c in centers:
    ax.plot([0,c[0]],[0,c[1]],[0,c[2]],color='#b1b9c2',ls=':',lw=1,zorder=1)
def arrow3(start,v,color,lw=1.6):
    ax.quiver(*start,*v,color=color,linewidth=lw,arrow_length_ratio=.12,normalize=False,zorder=9)
for i in range(4):
    c=centers[i]; col=colors[i]
    # 外框圆环的平面法向为框架轴g；内转子圆盘法向为自旋轴s。
    t=np.linspace(0,2*np.pi,100)
    a=s[:,i]; b=np.cross(g[:,i],a)
    ring=c[:,None]+.25*(a[:,None]*np.cos(t)+b[:,None]*np.sin(t))
    ax.plot(*ring,color=col,lw=2.3,zorder=5)
    d=np.cross(s[:,i],g[:,i])
    disk=c[:,None]+.17*(g[:,i,None]*np.cos(t)+d[:,None]*np.sin(t))
    ax.add_collection3d(Poly3DCollection([disk.T],facecolor=col,edgecolor=col,alpha=.28,lw=.8,zorder=6))
    arrow3(c-.40*g[:,i],.95*g[:,i],'#303b48')
    arrow3(c-.18*s[:,i],.78*s[:,i],col,2)
    gp=c+.62*g[:,i]
    ax.text(*gp,f'$g_{i+1}$',fontsize=13,color='#303b48',ha='center',zorder=15)
    ax.text(*(c-.40*np.array([0,0,1])),f'CMG {i+1}',fontsize=10,color=col,ha='center',zorder=15)
for v,lab in [(np.array([1.9,0,0]),'$x_B$'),(np.array([0,1.9,0]),'$y_B$'),(np.array([0,0,1.9]),'$z_B$')]:
    arrow3(np.zeros(3),v,'#6f7984',1.1);ax.text(*(v*1.08),lab,fontsize=13,ha='center')
ax.scatter([0],[0],[0],color='#303b48',s=13)
# beta位于z轴与g1轴之间（不是与xy平面之间）。
t=np.linspace(0,cl.prm.beta,40)
arc=.65*np.array([np.sin(t),np.zeros_like(t),np.cos(t)])
ax.plot(*arc,color='#303b48',lw=1.4,zorder=10)
ax.text(.49,-.08,.67,r'$\beta$',fontsize=16,zorder=15)
ax.set(xlim=(-1.65,1.8),ylim=(-1.7,1.8),zlim=(-.3,2.0))
ax.set_box_aspect((1,1,.7));ax.set_axis_off()
fig.text(.26,.94,'(a) 四台 SGCMG 金字塔构型',ha='center',fontsize=14,fontweight='bold')

top=fig.add_axes([.61,.23,.35,.65]);top.set_aspect('equal');top.axis('off')
top.set(xlim=(-2,2),ylim=(-2,2))
for v,lab in [((1.85,0),'$x_B$'),((0,1.85),'$y_B$')]:
    top.annotate('',xy=v,xytext=(0,0),arrowprops={'arrowstyle':'->','color':'#7d8792','lw':1.1})
    top.text(v[0]+.06,v[1]+.04,lab,fontsize=13)
top.plot(0,0,'o',ms=3,color='#333');top.text(.07,-.20,'O',fontsize=11)
top.add_patch(Circle((0,0),1.18,fill=False,ec='#c2cad2',ls='--',lw=1))
for i in range(4):
    th=i*np.pi/2; c=1.18*np.array([np.cos(th),np.sin(th)]);col=colors[i]
    top.plot(*c,'o',ms=20,mfc='white',mec=col,mew=1.7,zorder=4)
    top.text(*c,str(i+1),ha='center',va='center',fontsize=10,color=col,zorder=5)
    top.annotate('',xy=c*1.39,xytext=c*.35,arrowprops={'arrowstyle':'->','color':'#303b48','lw':1.7})
    top.annotate('',xy=c+.61*s[:2,i],xytext=c+.16*s[:2,i],arrowprops={'arrowstyle':'->','color':col,'lw':2})
    pos=c+.76*s[:2,i]
    top.text(*pos,f'$s_{i+1}(0)$',ha='center',va='center',fontsize=12,color=col)
    pos=c*1.50+.27*s[:2,i]
    top.text(*pos,f'$g_{{{i+1},xy}}$',ha='center',va='center',fontsize=12)
top.add_patch(Arc((0,0),1.15,1.15,theta1=8,theta2=82,color='#7d8792',lw=1))
top.text(.46,.45,'90°',fontsize=11,color='#59636e')
fig.text(.785,.94,'(b) 零框架角的俯视方向关系',ha='center',fontsize=14,fontweight='bold')
fig.text(.785,.16,r'$\theta_i=0^\circ,90^\circ,180^\circ,270^\circ$     $\gamma_i=0$',ha='center',fontsize=12)
fig.text(.50,.075,r'$\beta=\angle(g_i,z_B)=54.7356^\circ$     深灰箭头：框架轴     彩色箭头：零位自旋轴',ha='center',fontsize=11)
fig.text(.50,.025,'方向与项目模型一致；圆环和圆盘表示框架及转子，位置与大小仅作示意。',ha='center',fontsize=10,color='#5c6570')
for ext in ['png','svg','pdf']:
    fig.savefig(out/f'四台SGCMG金字塔构型.{ext}',dpi=300,facecolor='white')
print('Saved PNG, SVG, PDF')
