"""Momentum-exchange devices for the replay of the earlier C-group and T-group.

Only the CMG is a supported momentum-exchange device.  The earlier studies used
a reaction wheel; those runs exist in `legacy/` and in `results/legacy_rw/` and
are not part of this pipeline.

The device is a first-class axis:

    device  none | cmg
    control unified MPC | torque-level MPC + pseudo-inverse steering allocation
    model   nominal | deliberately mismatched aero coefficients

Every arm returns the same bundle interface:

    step(x, u, p, dt)   truth-model integrator
    ctrl.solve(x,t,traj) -> (u, info)
    view(x, u)          -> device observables: applied body moment, stored
                          momentum, peak gimbal rate, singularity index

`device="none"` uses the same 18-state model with `cmg_on=False`: the gimbals
are frozen at the home configuration and the CMG contributes neither torque nor
momentum, so it is the same vehicle minus the CMG rather than a different model.
"""
from __future__ import annotations

import copy

import numpy as np

from .cmg import CMGParams
from .cmg_control import LTVMPCCMG2
from .cmg_hier import CMGHierController
from .model6 import (VehicleParams, aero_moment, quat_to_euler)      # re-export
from .model6c import (VehicleCMGParams, aero_moment6c, make_cluster,
                      plant_step6c)
from .sim import _initial_state

# ---------------------------------------------------------------------------
# arm definitions  (C group = pitch task, T group = three-axis task)
# ---------------------------------------------------------------------------
ARMS = {
    # ---- C group ---------------------------------------------------------
    "C0": dict(device="cmg", controller="unified", ideal_both=True,
               label="C0 理想 CMG（真值与控制器同为 τ_g = 10 ms，无滞后失配）—— 上界"),
    "C1": dict(device="none", controller="unified",
               label="C1 仅气动舵面（CMG 在真值模型中断开）"),
    "C2": dict(device="cmg", controller="hier",
               label="C2 分层：力矩层 MPC + 伪逆操纵律分配 + CMG"),
    "C3": dict(device="cmg", controller="unified",
               label="C3 统一 MPC + CMG（统一优化 δ_c 与 M_cmg）"),
    "C4": dict(device="cmg", controller="unified", mismatch=1.2,
               label="C4 统一 MPC + CMG，控制器气动系数偏 20%（真值仍标称）"),
    # ---- T group ---------------------------------------------------------
    "T1": dict(device="none", controller="unified",
               label="T1 无动量交换装置，仅舵面"),
    "T3": dict(device="cmg", controller="unified",
               label="T3 统一 MPC 三轴 + CMG"),
}

# "ideal actuator" bound: both the truth model and the controller get a 10 ms
# gimbal lag (nominal 30 ms).  dt_plant = 5 ms and the internal rollouts use
# 10 ms steps, so 10 ms is the smallest lag the explicit integrator can carry.
IDEAL_TAU_G = 0.01

COEFS = ("Cm_alpha", "Cm_q", "Cm_de", "Cl_beta", "Cl_p", "Cl_r", "Cl_da",
         "Cl_dr", "Cn_beta", "Cn_p", "Cn_r", "Cn_da", "Cn_dr")


# ---------------------------------------------------------------------------
def _mismatched(veh: VehicleParams, factor: float) -> VehicleParams:
    """Controller aero model with every stability/control coefficient off by
    `factor`; the plant keeps the nominal coefficients."""
    out = copy.deepcopy(veh)
    for k in COEFS:
        setattr(out, k, getattr(veh, k) * float(factor))
    return out


def _cmg_view(prm):
    cl = make_cluster(prm)

    def view(x, u):
        g = x[10:14]
        A = cl.jacobian(g)
        return dict(dev_M=cl.torque(g, x[14:18]),
                    dev_H=cl.momentum(g),
                    dev_rate=float(np.max(np.abs(x[14:18]))),
                    sigma=float(cl.sigma_min(A)))
    return view


def build(cfg, arm: str):
    """Construct one arm: truth model, controller, initial state, observer."""
    if arm not in ARMS:
        raise KeyError("unknown arm %r; known: %s" % (arm, ", ".join(ARMS)))
    spec = ARMS[arm]
    cmg_on = spec["device"] == "cmg"
    mism = float(spec.get("mismatch", 1.0) or 1.0)

    cmg_base = CMGParams.from_config(cfg)
    cmg = (CMGParams.from_config(cfg, tau_g=IDEAL_TAU_G)
           if spec.get("ideal_both") else cmg_base)
    prm_plant = VehicleCMGParams.from_config(cfg, cmg=cmg, cmg_on=cmg_on)
    veh_ctrl = _mismatched(prm_plant.veh, mism)
    prm_ctrl = VehicleCMGParams(veh=veh_ctrl, cmg=cmg, cmg_on=cmg_on)
    cl_plant = make_cluster(prm_plant)
    if spec["controller"] == "hier":
        ctrl = CMGHierController(prm_ctrl, cfg)
    else:
        ctrl = LTVMPCCMG2(prm_ctrl, cfg, make_cluster(prm_ctrl))
    return dict(arm=arm, kind="cmg", label=spec["label"],
                prm_plant=prm_plant, prm_ctrl=prm_ctrl, cluster=cl_plant,
                step=lambda x, u, p, dt: plant_step6c(x, u, p, dt, prm_plant,
                                                      cl_plant, substeps=1)[0],
                ctrl=ctrl, view=_cmg_view(prm_plant), nu=7)


def initial_state(bundle, traj):
    """Trimmed start state, identical in every arm (same de, zero rates)."""
    return _initial_state(traj, bundle["prm_plant"])


__all__ = ["ARMS", "build", "initial_state", "aero_moment", "aero_moment6c",
           "quat_to_euler", "VehicleParams"]
