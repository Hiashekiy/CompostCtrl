"""Export future reference/environment schedules, not controller solutions."""
from pathlib import Path
import sys,numpy as np
from scipy.io import savemat
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from nearspace.torque_gap_ocp import _casadi
_casadi()
from nearspace.config import load_config
from nearspace.torque_gap_scenario import Scenario
case=ROOT/'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml'
if '--config' in sys.argv:case=Path(sys.argv[sys.argv.index('--config')+1]).resolve()
scn=Scenario(load_config(str(case)))
times=np.arange(int(round((scn.t_end-scn.t_start)/scn.dt_ctrl))+1)*scn.dt_ctrl
plans=[scn.plan(scn.t_start+t) for t in times]
data={'time':times,'x0':scn.initial_state()}
for name,group,key in [('q_ref','cond','q_ref'),('omega_ref','cond','omega_ref'),('tau_d','cond','tau_d'),
    ('tau_min','box','tau_min'),('tau_max','box','tau_max'),('tau_nom','mod','tau_nom'),('gap','mod','d')]:
    data[name]=np.stack([np.asarray(p[group][key]).T for p in plans],axis=2)
out=ROOT/'simulink/data/online_plan_bank.mat';savemat(out,data);print('ONLINE_PLAN_BANK_PASS',len(times))
