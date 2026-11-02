"""Regenerate the 14 paper experiment images without running simulations."""
from pathlib import Path
import shutil
import plot_no_cmg_comparison as ab
import analyze_prediction_mechanism as mechanism

ROOT=Path(__file__).resolve().parents[1]
ab.RES=ROOT/'results/torque_gap_no_cmg_comparison_rti2'
paper=ROOT/'论文/figures'
titles=['俯仰正30度','俯仰负30度','俯仰10度','俯仰45度','滚转30度','偏航30度','俯仰30滚转20度','连续机动']
number=0
for phase in ['P0','P1']:
    for sc,a,b in ab.collect(phase):
        number+=1
        src=ab.FIG/(phase+'_rti2')/(sc+'_AB.png')
        ab.fig_scenario(phase,sc,a,b,src)
        title=titles[ab.SCENARIO_ORDER.index(sc)]
        dest=paper/f'图4-{number}_{phase}_{title}.png'
        for target in [dest,paper/'修订'/f'{phase}_{sc}.png']:
            target.parent.mkdir(exist_ok=True,parents=True)
            shutil.copy2(src,target)
            shutil.copy2(src.with_suffix('.pdf'),target.with_suffix('.pdf'))
for tag,num,title in [('C',12,'C_无未来机动参考'),('A',13,'A_完整预测时域'),('B',14,'B_短预测时域')]:
    g=mechanism.load(tag);st=mechanism.stats(g)
    png,pdf=mechanism.plot_group(g,st)
    dest=paper/f'图4-{num}_{title}.png'
    shutil.copy2(png,dest);shutil.copy2(pdf,dest.with_suffix('.pdf'))
print('Refreshed 14 experiment figures and vector PDFs.')
