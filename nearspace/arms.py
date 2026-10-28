"""Momentum-exchange device arms for the C0-C4 / T1-T3 replay.

The earlier C/T studies were built around a **reaction-wheel** device.  This
module makes the device a first-class axis so the same experiment groups can be
replayed with the **CMG** and compared against the original wheel:

    device    none  | rw (3 orthogonal wheels) | cmg (4-unit pyramid)
    control   unified MPC | torque-level MPC + pseudo-inverse allocation
    model     nominal | deliberately mismatched aero coefficients

Every arm returns the same bundle interface:

    x0                 initial state (16 states for rw/none, 18 for cmg)
    step(x, u, p, dt)  truth-model integrator
    ctrl.solve(x,t,traj) -> (u, info)
    view(x, u)         -> device observables: applied body moment, stored
                          momentum, rate, singularity index (all body frame)

The device is part of the TRUTH model, so the truth models of different device
arms are not identical - that is the point of the comparison, not a fairness
defect.  What IS held identical across arms: atmosphere, trajectory/reference,
initial state, dt_ctrl/dt_plant, aero coefficients of the plant, cost weights.
"""
from __future__ import annotations

import copy

import numpy as np

from .cmg import CMGParams
from .cmg_control import LTVMPCCMG2
from .cmg_hier import CMGHierController
from .model6 import (VehicleParams, aero_moment, plant_step3, quat_to_euler)
from .model6c import (VehicleCMGParams, aero_moment6c, make_cluster,
                      plant_step6c)
from .mpc3 import LTVMPC3
from .sim import _initial_state

# ---------------------------------------------------------------------------
# arm definitions
# ---------------------------------------------------------------------------
ARMS = {
    # ---- C group (pitch-axis task, direction-of-flight study) ------------
    "C0_rw": dict(device="rw", controller="unified", ideal_both=True,
                  label="C0 理想执行机构（真值与控制器同为 10 ms 滞后），反作用轮 —— 上界"),
    "C0_cmg": dict(device="cmg", controller="unified", ideal_both=True,
                   label="C0' 理想 CMG（真值与控制器同为 τ_g = 10 ms）—— 上界"),
    "C1": dict(device="none", controller="unified",
               label="C1 仅气动舵面（无动量交换装置）"),
    "C2_cmg": dict(device="cmg", controller="hier",
                   label="C2' 分层：力矩层 MPC + 伪逆操纵律分配 + CMG"),
    "C3_rw": dict(device="rw", controller="unified",
                  label="C3 统一 MPC + 反作用轮（原 C3）"),
    "C3_cmg": dict(device="cmg", controller="unified",
                   label="C3' 统一 MPC + CMG（把装置换成 CMG）"),
    "C4_cmg": dict(device="cmg", controller="unified", mismatch=1.2,
                   label="C4' 统一 MPC + CMG，控制器气动系数偏 20%"),
    # ---- T group (three-axis task) ---------------------------------------
    "T1": dict(device="none", controller="unified",
               label="T1 无动量交换装置，仅舵面"),
    "T3_rw": dict(device="rw", controller="unified",
                  label="T3 统一 MPC 三轴 + 反作用轮（原 T3）"),
    "T3_cmg": dict(device="cmg", controller="unified",
                   label="T3' 统一 MPC 三轴 + CMG（把装置换成 CMG）"),
}

# "ideal actuator" bound: both the truth model and the controller get the faster
# first-order lag, so this is a genuine upper bound rather than a model-mismatch
# test (the original C0 only idealised the controller model, which is ill-posed
# on the lightly damped roll axis).  dt_plant = 5 ms and the rollouts use
# 10 ms steps, so 10 ms is the smallest lag the explicit integrator can carry.
IDEAL_TAU_S = 0.01
IDEAL_TAU_W = 0.01
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


def _rw_view(x, u):
    return dict(dev_M=-x[10:13],              # wheel reaction on the body
                dev_H=x[13:16],
                dev_rate=0.0,                 # no gimbal in a wheel
                sigma=float("nan"))


def build(cfg, arm: str):
    """Construct one arm: truth model, controller, initial state, observer."""
    if arm not in ARMS:
        raise KeyError("unknown arm %r; known: %s" % (arm, ", ".join(ARMS)))
    spec = ARMS[arm]
    device, mode = spec["device"], spec["controller"]
    mism = float(spec.get("mismatch", 1.0) or 1.0)
    ideal = bool(spec.get("ideal_both"))

    if device == "cmg":
        cmg_base = CMGParams.from_config(cfg)
        cmg_ideal = (CMGParams.from_config(cfg, tau_g=IDEAL_TAU_G) if ideal
                     else cmg_base)
        prm_plant = VehicleCMGParams.from_config(cfg, cmg=cmg_ideal, cmg_on=True)
        veh_ctrl = _mismatched(prm_plant.veh, mism)
        prm_ctrl = VehicleCMGParams(veh=veh_ctrl, cmg=cmg_ideal, cmg_on=True)
        cl_plant = make_cluster(prm_plant)
        if mode == "hier":
            ctrl = CMGHierController(prm_ctrl, cfg)
        else:
            ctrl = LTVMPCCMG2(prm_ctrl, cfg, make_cluster(prm_ctrl))
        return dict(arm=arm, kind="cmg", label=spec["label"],
                    prm_plant=prm_plant, prm_ctrl=prm_ctrl,
                    cluster=cl_plant,
                    step=lambda x, u, p, dt: plant_step6c(x, u, p, dt, prm_plant,
                                                          cl_plant, substeps=1)[0],
                    ctrl=ctrl, view=_cmg_view(prm_plant),
                    x0=None, nu=7)

    # ---- wheel / no device ------------------------------------------------
    wheel_on = (device == "rw")
    if mode == "hier":
        raise ValueError("the hierarchical arm only exists for the CMG device")
    if ideal:
        prm_plant = VehicleParams.from_config(cfg, wheel_on=wheel_on,
                                             tau_s=IDEAL_TAU_S, tau_w=IDEAL_TAU_W)
        prm_ctrl = prm_plant
    else:
        prm_plant = VehicleParams.from_config(cfg, wheel_on=wheel_on)
        prm_ctrl = prm_plant
    if mism != 1.0:
        prm_ctrl = _mismatched(prm_ctrl, mism)
    ctrl = LTVMPC3(prm_ctrl, cfg)
    return dict(arm=arm, kind="rw", label=spec["label"],
                prm_plant=prm_plant, prm_ctrl=prm_ctrl, cluster=None,
                step=lambda x, u, p, dt: plant_step3(x, u, p, dt, prm_plant,
                                                     substeps=1)[0],
                ctrl=ctrl, view=_rw_view, x0=None, nu=6)


def initial_state(bundle, traj):
    """Trimmed start state, identical in every arm (same de, zero rates)."""
    if bundle["kind"] == "cmg":
        return _initial_state(traj, bundle["prm_plant"])
    prm = bundle["prm_plant"]
    de = -prm.Cm_alpha / prm.Cm_de * (traj.pitch_ref(0.0) - traj.gamma(0.0))
    return np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0.0, 0.0],
                           np.zeros(3), np.zeros(3)])


__all__ = ["ARMS", "build", "initial_state", "VehicleParams", "aero_moment",
           "aero_moment6c", "quat_to_euler"]
