"""Export native migration parameters and subsystem oracles, without MATLAB.

Run from any directory: python simulink/py/export_native_baseline.py
The generated data are Python references, not Simulink acceptance results.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.io import savemat

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from nearspace.config import load_config, read_yaml
from nearspace.cmg import CMGCluster, CMGParams
from nearspace.model6 import VehicleParams, aero_moment, euler_to_quat
from nearspace.torque_gap_authority import natural_moment_v, surface_matrix
from nearspace.torque_gap_model import TorquePlantParams, dynamics_tg


def plain(value):
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml')
    args = parser.parse_args()
    raw = read_yaml(str(args.config))
    cfg = load_config(str(args.config))
    veh, cmg = VehicleParams.from_config(cfg), CMGParams.from_config(cfg)
    cl = CMGCluster(cmg)
    out = ROOT / 'simulink/data/native_baseline' / args.config.stem
    out.mkdir(parents=True, exist_ok=True)
    # Store each inherited source hash; a resolved root YAML alone is insufficient.
    sources = []
    import yaml
    source = args.config.resolve()
    while True:
        sources.append({'path': source.relative_to(ROOT).as_posix(), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest()})
        parent = (yaml.safe_load(source.read_text(encoding='utf-8')) or {}).get('base')
        if not parent:
            break
        source = (source.parent / parent).resolve()
    params = dict(vehicle=asdict(veh), cmg=asdict(cmg), J=np.diag([veh.Jx, veh.Jy, veh.Jz]),
                  g_axis=cl.g_axis, s0=cl.s0, gxs0=cl.gxs0,
                  dt_ctrl=raw['torque_gap']['dt_ctrl'], resolved_config=raw)
    (out / 'vehicle_params.json').write_text(json.dumps(plain(params), indent=2, ensure_ascii=False), encoding='utf-8')
    # Keep arbitrary YAML (including null) out of MAT; full configuration is JSON.
    savemat(out / 'vehicle_params.mat', {k: v for k, v in params.items() if k != 'resolved_config'})
    (out / 'config_resolved.json').write_text(json.dumps(raw, indent=2, ensure_ascii=False), encoding='utf-8')
    rng = np.random.default_rng(20261102)
    gamma = np.vstack([np.zeros((1, 4)), rng.uniform(-4*np.pi, 4*np.pi, (64, 4)),
                       np.full((1, 4), 2*np.pi-1e-6), np.full((1, 4), 2*np.pi+1e-6)])
    nu = rng.uniform(-cmg.gdot_max, cmg.gdot_max, gamma.shape)
    H = np.array([cl.momentum(g) for g in gamma])
    A = np.array([cl.jacobian(g) for g in gamma])
    torque = np.array([cl.torque(g, n) for g, n in zip(gamma, nu)])
    eps = 1e-6
    fd = np.array([np.column_stack([(cl.momentum(g+eps*np.eye(4)[i])-cl.momentum(g-eps*np.eye(4)[i]))/(2*eps) for i in range(4)]) for g in gamma])
    fd_error = float(np.max(abs(fd-A)))
    periodic_error = float(max(np.max(abs(cl.momentum(g+2*np.pi)-cl.momentum(g))) for g in gamma))
    assert fd_error < 1e-6 and periodic_error < 1e-10
    savemat(out / 'cmg_mapping_reference.mat', dict(gamma=gamma, nu=nu, H=H, A=A, torque=torque,
               Hdot=-torque, axis_order='sample,row,column'))
    # Algebraic derivation checked against the existing P0 truth derivative.
    prm = TorquePlantParams(veh, cmg)
    coupling_error = 0.0
    for g, n in zip(gamma, nu):
        w = rng.uniform(-0.3, 0.3, 3)
        ta, td = rng.normal(size=(2, 3))
        x = np.r_[euler_to_quat(*rng.uniform(-0.5, 0.5, 3)), w, g]
        actual = dynamics_tg(x, np.r_[ta, n], td, prm, cl)[4:7]
        equivalent = -cl.jacobian(g) @ n - np.cross(w, cl.momentum(g))
        expected = prm.Jinv @ (ta+td+equivalent-np.cross(w, prm.J @ w))
        coupling_error = max(coupling_error, float(np.max(abs(actual-expected))))
    assert coupling_error < 1e-12
    # Aerodynamic linear-derivative data: not CFD or measured tables.
    records = []
    aero_error = 0.0
    for _ in range(96):
        rho, speed = rng.uniform(1e-5, 0.02), rng.uniform(100, 4000)
        alpha, beta = rng.uniform(-0.3, 0.3, 2)
        w = rng.uniform(-0.2, 0.2, 3)
        delta = rng.uniform(-1, 1, 3)*veh.delta_max
        vb = speed*np.array([np.cos(beta)*np.cos(alpha), np.sin(beta), np.cos(beta)*np.sin(alpha)])
        x, p = np.r_[1., 0., 0., 0., w, delta, np.zeros(6)], np.r_[rho, vb, np.zeros(3)]
        qbar = 0.5*rho*speed**2
        natural = natural_moment_v(qbar, alpha, beta, w, speed, veh)
        incremental = surface_matrix(qbar, veh) @ delta
        total = aero_moment(x, p, veh)
        aero_error = max(aero_error, float(np.max(abs(total-natural-incremental))))
        records.append(np.r_[rho, speed, alpha, beta, w, delta, natural, incremental, total])
    assert aero_error < 1e-7
    np.savetxt(out / 'aerodynamic_reference.csv', records, delimiter=',', comments='',
       header='rho,V,alpha,beta,p,q,r,de,da,dr,nat_L,nat_M,nat_N,surface_L,surface_M,surface_N,total_L,total_M,total_N')
    report = dict(status='PYTHON_REFERENCE_VALIDATED_SIMULINK_PENDING', seed=20261102, sources=sources,
         samples_cmg=len(gamma), samples_aerodynamic=len(records), jacobian_fd_max_error=fd_error,
         momentum_periodic_max_error=periodic_error, equivalent_moment_acceleration_error=coupling_error,
         aerodynamic_decomposition_error=aero_error)
    (out / 'validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    print(out)


if __name__ == '__main__':
    main()
