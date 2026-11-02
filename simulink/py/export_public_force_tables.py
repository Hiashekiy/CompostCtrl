"""Public Betts shuttle-like force polar, explicitly a surrogate, not vehicle data."""
from pathlib import Path
import json
import numpy as np
from scipy.io import savemat
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'simulink/data/public_surrogate';out.mkdir(parents=True,exist_ok=True)
alpha_deg=np.linspace(-40,40,801)
CL=-.20704+.029244*alpha_deg
CD=.07854-.0061592*alpha_deg+.000621408*alpha_deg**2
assert np.all(CD>0)
savemat(out/'force_polar.mat',dict(alpha_rad=np.deg2rad(alpha_deg),CL=CL,CD=CD,CY=np.zeros_like(CL)))
np.savetxt(out/'force_polar.csv',np.c_[alpha_deg,np.deg2rad(alpha_deg),CL,CD],delimiter=',',
 header='alpha_deg,alpha_rad,CL,CD',comments='')
metadata=dict(name='Betts shuttle-like reentry surrogate',source='https://jump.dev/JuMP.jl/stable/tutorials/nonlinear/space_shuttle_reentry_trajectory/',
 corroborating_source='https://openmdao.github.io/dymos/examples/reentry/reentry.html',
 original_reference='Betts, Practical Methods for Optimal Control and Estimation Using Nonlinear Programming (2010)',
 coefficient_angle_unit='degree; Simulink table breakpoint unit radian',
 chosen_alpha_domain_deg=[-40,40],domain_note='Chosen guard interval inside the public Dymos benchmark control bounds [-90,90] deg; not experimental vehicle validity',
 limitations=['Not identified for this 1200 kg vehicle','No Mach/Reynolds dependence','CY=0 is an added symmetry assumption; no lateral-force validation',
 'Force polar only; existing moment model remains separate','Zero thrust represents unpowered glide, not a powered vehicle engine',
 'Outside alpha domain must raise a diagnostic; do not silently extrapolate'],
 propulsion='Unpowered glide: thrust=0',status='PUBLIC_SURROGATE_FOR_ENGINEERING_INTEGRATION_ONLY')
(out/'provenance.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
print('PUBLIC_FORCE_POLAR_EXPORTED')
