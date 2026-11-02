"""Sample the existing frozen MSIS2 configuration for native lookup blocks."""
from pathlib import Path
import sys,numpy as np
from scipy.io import savemat
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from nearspace.torque_gap_ocp import _casadi
_casadi()
from nearspace.config import load_config
from nearspace.torque_gap_scenario import Scenario
scn=Scenario(load_config(str(ROOT/'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml')))
h=np.arange(0,120001,250,dtype=float);rho=np.array([scn.atm.rho(v) for v in h])
savemat(ROOT/'simulink/data/native_environment.mat',dict(height_m=h,rho=rho,h0=scn.traj.h(scn.t_start),V_ned0=scn.traj.V_ned(scn.t_start)))
assert np.all(np.isfinite(rho)) and np.all(rho>0);print('NATIVE_ENVIRONMENT_TABLE_PASS')
