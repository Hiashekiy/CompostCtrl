from pathlib import Path
import sys,numpy as np
from scipy.io import savemat
from scipy.optimize import lsq_linear
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from nearspace.config import load_config
from nearspace.model6 import VehicleParams
from nearspace.torque_gap_authority import surface_matrix
v=VehicleParams.from_config(load_config(str(ROOT/'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml')))
rng=np.random.default_rng(20261102);M=[];targets=[];lower=[];upper=[];cost=[]
for k in range(100):
    matrix=surface_matrix(10**rng.uniform(-3,4),v)
    prev=rng.uniform(-1,1,3)*v.delta_max
    lo=np.maximum(-v.delta_max,prev-.2*v.delta_dot_max);hi=np.minimum(v.delta_max,prev+.2*v.delta_dot_max)
    target=rng.normal(size=3)*max(np.linalg.norm(matrix),1)*.5
    sol=lsq_linear(matrix,target,bounds=(lo,hi),method='bvls',tol=1e-10)
    M.append(matrix);targets.append(target);lower.append(lo);upper.append(hi);cost.append(np.linalg.norm(matrix@sol.x-target)**2)
out=ROOT/'simulink/data/allocation_reference.mat'
savemat(out,dict(M=np.array(M).transpose(1,2,0),target=np.array(targets).T,lower=np.array(lower).T,upper=np.array(upper).T,cost=cost))
print('ALLOCATION_REFERENCE_EXPORTED')
