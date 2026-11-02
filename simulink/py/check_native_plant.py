"""Independent RK4 angular dynamics using logged actual drive states."""
from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from nearspace.config import load_config
from nearspace.torque_gap_model import TorquePlantParams
cfg=load_config(str(ROOT/'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml'))
p=TorquePlantParams.from_config(cfg);cluster=p.cluster();out=ROOT/'simulink/results/native_plant';report={}
for enabled in [False,True]:
    name='Plant_6DOF_CMG' if enabled else 'Plant_6DOF_SurfaceOnly'
    data=np.loadtxt(out/(name+'.csv'),delimiter=',');w=data[0,1:4].copy();values=[w.copy()]
    for k in range(len(data)-1):
        dt=data[k+1,0]-data[k,0];g=data[k,7:11];nu=data[k,11:15]
        H=cluster.momentum(g) if enabled else np.zeros(3)
        torque=cluster.torque(g,nu) if enabled else np.zeros(3)
        def f(value):return p.Jinv@(np.array([2.1,3.2,-.9])+torque-np.cross(value,p.J@value+H))
        a=f(w);b=f(w+.5*dt*a);c=f(w+.5*dt*b);d=f(w+dt*c);w+=dt/6*(a+2*b+2*c+d);values.append(w.copy())
    err=float(np.max(abs(np.asarray(values)-data[:,1:4])))
    assert err<2e-5,(name,err)
    report[name]={'omega_error_rad_s':err,'samples':len(data),'tolerance':2e-5}
(out/'independent_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
