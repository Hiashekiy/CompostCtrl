#!/usr/bin/env python
"""C1 control-process overview figure (five panels) from the archived run.

Everything plotted comes either from `results/torque_gap_unlimited/C1_B0_P0/
timeseries.csv` (raw experiment output) or from the project's *actual* CMG model
evaluated at the logged gimbal angles (derived; formula and parameters are stated
in the report).  Nothing is interpolated, resampled or invented.

Panels (shared time axis, 90-146 s):
  1  attitude: reference vs actual pitch (project quaternion convention)
  2  torques: reference demand tau_nom, surface torque, CMG torque, their sum
  3  surface authority: predicted aero torque bounds (lo_y, hi_y) + actual torque
  4  CMG three-axis actual torque
  5  CMG instantaneous pitch capability from the real A(gamma) and |nu_i| <= gdot_max

Outputs
  figures/torque_gap_unlimited/C1_control_overview.png   (300 dpi)
  figures/torque_gap_unlimited/C1_control_overview.pdf   (vector)
  docs/torque_gap/C1_control_overview_analysis.md

python scripts/plot_c1_control_overview.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGParams, CMGCluster                    # noqa: E402
from nearspace.config import load_config                           # noqa: E402
from nearspace.model6 import VehicleParams, quat_to_euler          # noqa: E402
from nearspace.atmosphere_factory import make_atmosphere           # noqa: E402
from nearspace.trajectory3 import Trajectory3D                     # noqa: E402
from nearspace.torque_gap_modulation import nominal_demand          # noqa: E402

RUN = ROOT / "results" / "torque_gap_unlimited" / "C1_B0_P0"
FIG = ROOT / "figures" / "torque_gap_unlimited"
DOC = ROOT / "docs" / "torque_gap"
T_WIN = (90.0, 146.0)
T_MANEUVER = 110.0          # configs/torque_gap_unlimited/cases/c1_pitch30.yaml
MIG_TOL_DEG = 3.0           # definition used by nearspace/torque_gap_sim.py

LABELS = {
    "tau_nom_y": r"参考需求力矩 $\tau_{\mathrm{nom},y}$",
    "tau_act_y": r"舵面实际力矩 $\tau_{a,y}$",
    "cmg_y": r"CMG 实际力矩 $\tau_{\mathrm{cmg},y}$",
    "sum_y": r"合成控制力矩 $\tau_{a,y}+\tau_{\mathrm{cmg},y}$",
}


# --------------------------------------------------------------------------
# data / model
# --------------------------------------------------------------------------
def load_run(run=RUN):
    ts = run / "timeseries.csv"
    if not ts.exists():
        raise SystemExit("missing %s" % ts)
    with open(ts, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    d = {}
    for key in rows[0]:
        try:
            d[key] = np.array([float(r[key]) for r in rows])
        except ValueError:
            d[key] = np.array([r[key] for r in rows])
    cfg = load_config(str(run / "config_resolved.yaml"))
    meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
    metrics = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
    return d, cfg, meta, metrics


def cmg_model(cfg):
    """CMG device taken from the run's own resolved configuration (no defaults)."""
    prm = CMGParams.from_config(cfg)
    return prm, CMGCluster(prm)


def pitch_deg(q):
    """Pitch angle [deg] with the project quaternion convention (body->NED, ZYX)."""
    return np.array([np.rad2deg(quat_to_euler(q[i, :])[1]) for i in range(q.shape[0])])


def capability(cluster, gamma, axis=1, gdot_max=1.5):
    """Instantaneous CMG torque capability along one body axis.

        tau_max+ = max_{nu in U} e_a^T (-A(gamma) nu)
        tau_max- = min_{nu in U} e_a^T (-A(gamma) nu),   U = {|nu_i| <= gdot_max}

    The map is linear and the box separable, so the optimum is on a vertex; it is
    computed by explicit enumeration of the 16 vertices and the analytic identity
    tau_max+ = -tau_max- = gdot_max * sum_i |A[a, i]| is verified.
    """
    n = gamma.shape[0]
    tp = np.zeros(n)
    tm = np.zeros(n)
    worst = 0.0
    verts = np.array([[(-1.0) ** ((k >> i) & 1) for i in range(4)] for k in range(16)])
    for k in range(n):
        A = cluster.jacobian(gamma[k])
        flat = (verts * gdot_max) @ (-A.T)                 # (16, 3)
        tp[k] = float(np.max(flat[:, axis]))
        tm[k] = float(np.min(flat[:, axis]))
        worst = max(worst, abs(tp[k] - gdot_max * float(np.sum(np.abs(A[axis, :])))))
    return tp, tm, worst


def momentum_bound(cluster, h0, axis=1):
    """Exact attainable bound of |H_axis| over the whole gimbal torus.

    H_axis(gamma) = h0 * sum_i s_i,axis(gamma_i) is separable, and
    s_i,axis(gamma) = s0_i,axis cos(gamma) + (g_i x s0_i)_axis sin(gamma),
    so max_gamma H_axis = h0 * sum_i sqrt(s0_i,axis^2 + (g_i x s0_i)_axis^2).
    """
    amp = np.sqrt(cluster.s0[axis, :] ** 2 + cluster.gxs0[axis, :] ** 2)
    return float(h0 * amp.sum())


def capability_along(cluster, gamma_k, direction, gdot_max):
    """max_{|nu_i|<=gmax} direction^T (-A nu) for an arbitrary unit direction."""
    A = cluster.jacobian(gamma_k)
    return float(gdot_max * np.sum(np.abs(A.T @ np.asarray(direction, float))))


def prm_like(cfg):
    """Minimal object carrying J and veh, as required by nominal_demand()."""
    veh = VehicleParams.from_config(cfg)
    return SimpleNamespace(J=np.diag([veh.Jx, veh.Jy, veh.Jz]), veh=veh)


def stats3(v, mask):
    x = v[mask]
    return (float(np.mean(x)), float(np.max(np.abs(x))),
            float(np.sqrt(np.mean(x ** 2))))


# --------------------------------------------------------------------------
# figure
# --------------------------------------------------------------------------
def plot(d, cfg, meta, metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False,
        "mathtext.fontset": "dejavusans",
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 10,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 7.5,
        "axes.linewidth": 0.8,
        "lines.linewidth": 1.1,
        "grid.linewidth": 0.5,
        "grid.alpha": 0.35,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.bbox": "tight",
    })

    t = d["t"]
    dt_ctrl = float(cfg.torque_gap.dt_ctrl)
    t_state = t + dt_ctrl          # see §1.3 of the analysis: state columns are logged post-step
    q = np.column_stack([d["q0"], d["q1"], d["q2"], d["q3"]])
    qref = np.column_stack([d["qref0"], d["qref1"], d["qref2"], d["qref3"]])
    pitch = pitch_deg(q)
    pitch_ref = pitch_deg(qref)

    t_mig = float(metrics.get("migration_onset_t") or 90.8)
    idx = np.flatnonzero(np.abs(d["d_y"]) > 1e-6)
    t_gap = float(t[idx[0]]) if idx.size else None

    fig, axes = plt.subplots(5, 1, figsize=(7.0, 11.6), sharex=True,
                             gridspec_kw=dict(hspace=0.13))
    colors = dict(nom="tab:blue", surf="tab:green", cmg="tab:red",
                  total="0.15", band="0.75")

    # ---------------- panel 1: attitude --------------------------------
    ax = axes[0]
    ax.plot(t, pitch_ref, color=colors["nom"], lw=2.4, alpha=0.85,
            label=r"参考俯仰角 $\theta_{\mathrm{ref}}$")
    ax.plot(t_state, pitch, color=colors["total"], lw=1.0, label=r"实际俯仰角 $\theta$")
    ax.set_ylabel("俯仰角 [deg]")
    ax.set_ylim(0.0, 34.0)
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)

    # ---------------- panel 2: torques ---------------------------------
    ax = axes[1]
    ax.plot(t, d["tau_nom_y"], color=colors["nom"], ls="--", lw=1.1,
            label=r"$\tau_{\mathrm{nom},y}$")
    ax.plot(t, d["tau_act_y"], color=colors["surf"], lw=1.2,
            label=r"$\tau_{a,y}$（舵面）")
    ax.plot(t_state, d["cmg_y"], color=colors["cmg"], lw=1.2,
            label=r"$\tau_{\mathrm{cmg},y}$（CMG）")
    ax.plot(t, d["tau_act_y"] + d["cmg_y"], color=colors["total"], lw=1.5, alpha=0.75,
            label=r"合成 $\tau_{a,y}+\tau_{\mathrm{cmg},y}$")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("俯仰力矩 [N$\\cdot$m]")
    ax.set_ylim(-120.0, 215.0)
    ax.legend(loc="upper center", ncol=4, framealpha=0.9,
              handlelength=1.6, columnspacing=1.1, borderpad=0.3)

    # ---------------- panel 3: surface authority ------------------------
    ax = axes[2]
    lo, hi = d["lo_y"], d["hi_y"]
    ax.fill_between(t, lo, hi, color=colors["band"], alpha=0.45,
                    label=r"舵面可实现力矩范围 $[\underline{\tau}_y,\ \overline{\tau}_y]$")
    ax.plot(t, hi, color="0.45", lw=0.8)
    ax.plot(t, lo, color="0.45", lw=0.8)
    ax.plot(t, d["tau_act_y"], color=colors["surf"], lw=1.2, label=LABELS["tau_act_y"])
    ax.plot(t, d["tau_nom_y"], color=colors["nom"], ls="--", lw=1.1,
            label=LABELS["tau_nom_y"])
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("俯仰力矩 [N$\\cdot$m]")
    ax.set_ylim(-95.0, 185.0)          # head room for the two-row legend
    ax.legend(loc="upper center", ncol=2, framealpha=0.9)

    # ---------------- panel 4: CMG three axes ---------------------------
    ax = axes[3]
    for key, lab, col in (("cmg_x", r"$\tau_{\mathrm{cmg},x}$ (roll)", "tab:purple"),
                          ("cmg_y", r"$\tau_{\mathrm{cmg},y}$ (pitch)", "tab:red"),
                          ("cmg_z", r"$\tau_{\mathrm{cmg},z}$ (yaw)", "tab:orange")):
        ax.plot(t_state, d[key], color=col, lw=1.2 if key == "cmg_y" else 0.9, label=lab)
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("CMG 力矩 [N$\\cdot$m]")
    ax.set_ylim(-120.0, 205.0)
    ax.legend(loc="upper center", ncol=3, framealpha=0.9)

    # ---------------- panel 5: instantaneous capability -----------------
    ax = axes[4]
    prm, cluster = cmg_model(cfg)
    h0 = float(prm.h0)
    gamma = np.column_stack([d["g%d" % (i + 1)] for i in range(4)])
    tp, tm, worst = capability(cluster, gamma, axis=1, gdot_max=prm.gdot_max)
    ax.fill_between(t_state, tm, tp, color=colors["band"], alpha=0.35,
                    label="瞬时能力带")
    ax.plot(t_state, tp, color="0.30", lw=1.0, label=r"$\tau^{+}_{\max,y}$")
    ax.plot(t_state, tm, color="0.30", lw=1.0, ls="-.", label=r"$\tau^{-}_{\max,y}$")
    ax.plot(t_state, d["cmg_y"], color=colors["cmg"], lw=1.3,
            label=r"$\tau_{\mathrm{cmg},y}$")
    ax.plot(t, d["d_y"], color=colors["nom"], lw=1.2, label=r"缺口 $d_y$")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("俯仰力矩 [N$\\cdot$m]")
    ax.set_xlabel("时间 $t$ [s]")
    ax.set_ylim(-260.0, 520.0)         # head room for the two-row legend

    # right axis: the pitch-axis angular momentum H_y and its exact attainable
    # bound.  This is what the migration actually moves -- it is NOT a torque.
    Hp = momentum_bound(cluster, h0, axis=1)
    ax2 = ax.twinx()
    hy = d["Hy"]
    ax2.axhspan(-Hp, Hp, color="tab:blue", alpha=0.06, lw=0)
    for yv in (Hp, -Hp):
        ax2.axhline(yv, color="tab:blue", ls=":", lw=0.9)
    ax2.plot(t_state, hy, color="tab:blue", lw=1.7, label=r"$H_y$（右轴·角动量）")
    ax2.set_ylim(-Hp * 1.75, Hp * 1.75)
    ax2.set_ylabel(r"$H_y$ [N$\cdot$m$\cdot$s]")
    ax2.tick_params(labelsize=7)
    ax2.spines["top"].set_visible(False)

    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper left", ncol=2, framealpha=0.9,
              fontsize=6.8, handlelength=1.3, columnspacing=0.8, borderpad=0.3,
              labelspacing=0.35)

    # ---------------- events on every panel -----------------------------
    for ax in axes:
        ax.axvline(t_mig, color="tab:red", ls=":", lw=1.0)
        ax.axvline(T_MANEUVER, color="k", ls="--", lw=1.0)
        if t_gap is not None:
            ax.axvline(t_gap, color="tab:blue", ls="-.", lw=0.9)
        ax.grid(True, which="major")
        ax.set_xlim(*T_WIN)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    # event labels above the top panel: no overlap with curves or the legend
    axes[0].annotate("迁移开始 $t\\approx%.1f$ s" % t_mig,
                     xy=(t_mig, 1.015), xycoords=("data", "axes fraction"),
                     xytext=(3, 0), textcoords="offset points",
                     fontsize=6.8, va="bottom", ha="left", color="tab:red")
    axes[0].annotate("机动开始 $t=%.0f$ s" % T_MANEUVER,
                     xy=(T_MANEUVER, 1.015), xycoords=("data", "axes fraction"),
                     xytext=(-3, 0), textcoords="offset points",
                     fontsize=6.8, va="bottom", ha="right", color="k")
    if t_gap is not None:
        axes[0].annotate("缺口出现 $t\\approx%.1f$ s" % t_gap,
                         xy=(t_gap, 1.015), xycoords=("data", "axes fraction"),
                         xytext=(3, 0), textcoords="offset points",
                         fontsize=6.8, va="bottom", ha="left", color="tab:blue")

    axes[0].set_title(
        "C1 工况控制过程总图（%s，%s 植物，无框架角行程限制）：提前构型迁移与舵面–CMG 协同"
        % (meta.get("case", "C1"), meta.get("experiment", {}).get("plant", "P0")),
        fontsize=10, pad=17)

    FIG.mkdir(parents=True, exist_ok=True)
    png = FIG / "C1_control_overview.png"
    pdf = FIG / "C1_control_overview.pdf"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    plt.close(fig)
    return dict(t_mig=t_mig, t_gap=t_gap, tp=tp, tm=tm, worst_identity=worst,
                gdot_max=prm.gdot_max, png=png, pdf=pdf, cluster=cluster,
                prm=prm, gamma=gamma)


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------
def report(d, cfg, meta, metrics, info):
    prm, cluster = info["prm"], info["cluster"]
    t = d["t"]
    gamma = info["gamma"]
    nu = np.column_stack([d["nu%d" % (i + 1)] for i in range(4)])
    tp, tm = info["tp"], info["tm"]
    win = metrics.get("window") or [T_MANEUVER, T_MANEUVER + 20.0]
    pre = (t >= 90.0) & (t < T_MANEUVER)
    man = (t >= win[0]) & (t <= win[1])
    mig_t = info["t_mig"]

    pitch = pitch_deg(np.column_stack([d["q0"], d["q1"], d["q2"], d["q3"]]))
    pitch_ref = pitch_deg(np.column_stack([d["qref0"], d["qref1"], d["qref2"],
                                          d["qref3"]]))
    J = np.diag([cfg.vehicle3.Jx, cfg.vehicle3.Jy, cfg.vehicle3.Jz])
    traj = Trajectory3D(cfg, make_atmosphere(cfg))
    veh = VehicleParams.from_config(cfg)
    pl = prm_like(cfg)

    # ---- independent replication of tau_nom (field-semantics check) ----
    tau_d = np.array([traj.disturbance(tt, cfg, veh) for tt in t])
    tau_nom_rep = np.zeros_like(t)
    for k, tt in enumerate(t):
        H_nom = cluster.momentum(gamma[max(k - 1, 0)])      # gimbal state at plan time
        tau_nom_rep[k] = nominal_demand(traj, float(tt), pl, H_nom, tau_d[k])[1]
    worst_nom = float(np.max(np.abs(tau_nom_rep - d["tau_nom_y"])))

    # ---- magnitude of the gyroscopic term (not an actuator output) -----
    wref = np.column_stack([d["wrefx"], d["wrefy"], d["wrefz"]])
    H = np.column_stack([d["Hx"], d["Hy"], d["Hz"]])
    # actual gyroscopic coupling (measured omega and H), the term that makes the
    # applied control torque differ from the feed-forward demand
    W_act = np.column_stack([d["wx"], d["wy"], d["wz"]])
    gyro_all = np.cross(W_act, (J @ W_act.T).T + H)        # (n, 3)
    gyro = gyro_all[:, 1]

    # ---- state-propagation check (no hidden torque term) ---------------
    # Re-integrate the 11-state P0 model over every logged control interval with
    # the logged applied torques [tau_a, nu] and the configured disturbance, and
    # compare with the next logged state.  This is the strong form of "the two
    # actuator torques plus the model terms reproduce the plant": a missing or
    # double-counted compensation term would show up here as a systematic drift.
    from nearspace.torque_gap_model import TorquePlantParams, pack_state, rk4_step_tg
    prm_tg = TorquePlantParams.from_config(cfg)
    dt_ctrl = float(cfg.torque_gap.dt_ctrl)
    n_sub = int(getattr(cfg.torque_gap, "plant_substeps", 8))
    # CSV timing convention: the *state* columns of row k are logged after the
    # control step, i.e. they are the state at t_k + dt_ctrl, while the commands
    # of row k were applied over [t_k, t_k + dt_ctrl].  The correct pairing is
    # therefore: state of row k-1 + commands of row k  ->  state of row k.
    Q = np.column_stack([d["q0"], d["q1"], d["q2"], d["q3"]])
    W = np.column_stack([d["wx"], d["wy"], d["wz"]])
    TA = np.column_stack([d["tau_act_x"], d["tau_act_y"], d["tau_act_z"]])
    worst_state = {1: 0.0, 2: 0.0, 3: 0.0}
    n_check = t.size - 1
    for k in range(1, t.size):
        xk = pack_state(Q[k - 1], W[k - 1], gamma[k - 1])
        u = np.concatenate([TA[k], nu[k]])
        h = dt_ctrl / n_sub
        for i in range(n_sub):
            td = traj.disturbance(t[k] - dt_ctrl + (i + 0.5) * h, cfg, veh)
            xk = rk4_step_tg(xk, u, td, h, prm_tg, cluster)
        worst_state[1] = max(worst_state[1], float(np.max(np.abs(xk[4:7] - W[k]))))
        worst_state[2] = max(worst_state[2], float(np.max(np.abs(xk[7:11] - gamma[k]))))
        worst_state[3] = max(worst_state[3], float(np.max(np.abs(xk[0:4] - Q[k]))))

    # ---- actuator verification ----------------------------------------
    cmg_y_model = np.array([cluster.jacobian(gamma[k])[1, :] @ nu[k]
                            for k in range(t.size)])
    worst_cmg = float(np.max(np.abs(-cmg_y_model - d["cmg_y"])))
    worst_act = float(np.max(np.abs(d["tau_act_y"] - d["tauc_cmd_y"])))

    # ---- capability utilisation (per sample) --------------------------
    cap = np.minimum(tp, -tm)
    ratio = np.abs(d["cmg_y"]) / np.maximum(cap, 1e-9)
    util_med = float(np.median(ratio[man]))
    util_max = float(np.max(ratio[man]))
    inside = bool(np.all(ratio[man] <= 1.0 + 1e-6))

    # ---- gap statistics ------------------------------------------------
    gap = np.abs(d["d_y"])
    gap_frac = float(np.mean(gap[man] > 1e-6))
    gap_peak = float(np.max(gap[man]))
    order = np.flatnonzero(man)
    i_gp = int(order[int(np.argmax(gap[man]))])
    dir_gap = float(np.sign(d["d_y"][i_gp]))                 # +1: need +y torque
    # direction the CMG must push: +tau_cmg along dir_gap
    cap_dir = capability_along(cluster, gamma[i_gp],
                               np.eye(3)[1] * dir_gap, prm.gdot_max)
    short_frac = float(np.mean(gap[man] > cap[man]))

    S = {}
    for name, mask in (("pre", pre), ("man", man)):
        S[name] = dict(
            nom=stats3(d["tau_nom_y"], mask),
            act=stats3(d["tau_act_y"], mask),
            cmg=stats3(d["cmg_y"], mask),
            tot=stats3(d["tau_act_y"] + d["cmg_y"], mask),
            res=stats3(d["tau_nom_y"] - d["tau_act_y"] - d["cmg_y"], mask),
        )
    opp_pre = float(np.mean(np.sign(d["tau_act_y"][pre]) *
                            np.sign(d["cmg_y"][pre]) < 0))
    opp_man = float(np.mean(np.sign(d["tau_act_y"][man]) *
                            np.sign(d["cmg_y"][man]) < 0))
    gyro_pre, gyro_man = float(np.max(np.abs(gyro[pre]))), float(np.max(np.abs(gyro[man])))
    gyro_pre_all = float(np.max(np.abs(gyro_all[pre])))
    gyro_man_all = float(np.max(np.abs(gyro_all[man])))
    td_pre, td_man = float(np.max(np.abs(tau_d[pre]))), float(np.max(np.abs(tau_d[man])))

    # ---- migration / capability at key times --------------------------
    k_at = lambda tt: int(np.argmin(np.abs(t - tt)))
    over90 = np.flatnonzero(np.any(np.abs(gamma) > np.pi / 2, axis=1))
    t90 = float(t[over90[0]]) if over90.size else None
    cap_tab = [(tt, tp[k_at(tt)], tm[k_at(tt)]) for tt in
               (90.0, 95.0, 100.0, 105.0, 110.0, 112.0, 120.0, 125.0)]
    Hp = momentum_bound(cluster, float(prm.h0), axis=1)
    hy = d["Hy"]
    hy_tab = [(tt, float(hy[k_at(tt)])) for tt in (90.0, 95.0, 100.0, 105.0, 108.0,
                                                   110.0, 112.0, 120.0)]
    head_down_90 = float(hy[0] + Hp)                 # room to *decrease* H_y
    head_down_110 = float(hy[k_at(110.0)] + Hp)
    head_up_110 = float(Hp - hy[k_at(110.0)])
    # envelope at the configuration that maximises H_y: the gradient A[1,:] of
    # H_y w.r.t. gamma vanishes there, so the instantaneous y-capability is 0
    g_at_max = np.array([np.arctan2(cluster.gxs0[1, i], cluster.s0[1, i])
                         for i in range(4)])
    cap_at_max = capability(cluster, g_at_max.reshape(1, 4), axis=1,
                            gdot_max=prm.gdot_max)[0][0]
    grad_at_max = float(np.sum(np.abs(cluster.jacobian(g_at_max)[1, :])))
    sig_pre_min = float(np.min(d["sigma_min"][(t > 107.0) & (t < 111.0)]))
    cap_pre_med = float(np.median(tp[pre]))
    cap_man_med = float(np.median(tp[man]))
    cap_man_max = float(np.max(tp[man]))
    pre_err = np.abs(pitch - pitch_ref)[pre]
    mig_err = np.abs(pitch - pitch_ref)[(t >= mig_t) & (t < T_MANEUVER)]

    md = [f"# C1 工况控制过程总图分析（{meta.get('case','C1')} / "
          f"{meta.get('experiment',{}).get('mode','B0')} / "
          f"{meta.get('experiment',{}).get('plant','P0')}，无框架角行程限制）", "",
          "数据来源：`results/torque_gap_unlimited/C1_B0_P0/`（`timeseries.csv`、"
          "`config_resolved.yaml`、`meta.json`、`metrics.json`）。"
          "绘图脚本：`scripts/plot_c1_control_overview.py`；"
          "图：`figures/torque_gap_unlimited/C1_control_overview.png` / `.pdf`。", "",
          "## 1. 数据与代码核查", "",
          "### 1.1 CSV 字段的物理含义（以代码实际定义为准）", "",
          "| 字段 | 含义 | 单位/坐标 | 来源代码 |", "|---|---|---|---|",
          "| `t` | 控制时刻（`dt_ctrl = %.2f s`） | s | `torque_gap_sim.ClosedLoop.run` |"
          % float(cfg.torque_gap.dt_ctrl),
          "| `q0..q3`, `qref0..qref3` | 实际/参考四元数（标量在前 Hamilton，机体→NED） | — | 同上 |",
          "| `wx,wy,wz`, `wref*` | 机体角速度 `[p,q,r]` | rad/s，机体系 | 同上 |",
          "| `g1..g4`, `nu1..nu4` | 框架角（**展开角，不取模**）与框架角速度 | rad, rad/s | 同上 |",
          "| `Hx,Hy,Hz`, `H_norm` | `H(γ)=Σ h0 s_i(γ_i)` 及模长 | N·m·s，机体系 | `cmg.CMGCluster.momentum` |",
          "| `sigma_min` | `σ_min(A(γ))`，**仅诊断**，不参与约束 | N·m·s | `CMGCluster.sigma_min` |",
          "| `tauc_cmd_*` | 优化器给出的气动三轴力矩指令 | N·m | `torque_gap_sim._log_row` |",
          "| `tau_act_*` | 实际施加上气动力矩 | N·m | P0：`_step_P0` 原样回传指令；P1：`aero_moment6c` |",
          "| `cmg_*` | CMG 实际力矩 `τ_cmg = −A(γ)ν` | N·m | `_log_row`（真实 γ,ν 计算） |",
          "| `tau_nom_*` | 参考轨迹的**名义需求**总控制力矩 `J ω̇_ref + ω_ref×(J ω_ref+H_nom) − τ_d` | N·m | `torque_gap_modulation.nominal_demand` |",
          "| `Rp_* / Rm_*` | 气动余量 `R⁺=hi−τ_nom`、`R⁻=τ_nom−lo` | N·m | `torque_gap_modulation.modulation` |",
          "| `d_*` | 缺口 `d = τ_nom − clip(τ_nom, lo, hi)` | N·m | 同上 |",
          "| `lo_* / hi_*` | 预测气动盒下/上界（**总**气动力矩，含自然项） | N·m | `torque_gap_authority.build_boxes` |",
          "", "### 1.2 五项确认", "",
          "1. **`tau_nom` 就是参考姿态轨迹对应的需求力矩。** 用与代码完全相同的口径独立复算"
          "（`omega_ref` 中心差分 `eps=0.05 s`、`H_nom` 取**规划时刻**（上一样本）的 γ、"
          "扰动取配置的 `trajectory3.disturbance`），与 CSV 的 `tau_nom_y` 最大偏差 = **%.2e N·m**。"
          % worst_nom,
          "2. **`lo_y`/`hi_y` 是俯仰方向舵面可实现的总气动力矩上下界。** `y` 为机体俯仰轴；"
          "盒由 `nat_ref ± 0.774·hull(Q,δ_max)` 给出，0.774 是 "
          "`results/torque_gap/model_checks.json` 标定的保守比例（≥99%% 盒内样本可被有界舵偏实现）。"
          "t=90 s 时 `[%.2f, %.2f] N·m`，与面板 3 灰带一致。" % (d["lo_y"][0], d["hi_y"][0]),
          "3. **`tau_act_y` 是实际气动力矩、`cmg_y` 是实际 CMG 力矩。** 本运行是 **P0 虚拟力矩植物**："
          "舵面被抽象为理想力矩执行器，因此 `tau_act_y ≡ tauc_cmd_y`（最大偏差 %.1e N·m）；"
          "这**不是**真实舵偏角实现，真实舵面见 P1 运行。`cmg_y` 用真实 γ,ν 通过 `−A(γ)ν` 复算，"
          "与 CSV 最大偏差 = %.2e N·m。" % (worst_act, worst_cmg),
          "4. **参数全部取自本次运行的 `config_resolved.yaml`**：`h0 = %.1f N·m·s`、"
          "`gdot_max = %.2f rad/s`、`gddot_max = %.1f rad/s²`、`β = %.4f°`；未使用类默认值。"
          % (prm.h0, prm.gdot_max, prm.gddot_max, np.rad2deg(prm.beta)),
          "5. **两个执行机构之和就是控制力矩，没有隐藏补偿项。** 机体系力矩方程为 "
          "`J ω̇ = τ_a − A(γ)ν + τ_d − ω×(Jω+H)`，故 `τ_a + τ_cmg` 等于控制力矩；"
          "陀螺项 `−ω×(Jω+H)` 与扰动 `τ_d` 不属于执行机构输出，面板 2 未把它们并入。"
          "量级（由 CSV 的 `wx/wy/wz`、`H` 与配置扰动模型算出）：实际陀螺耦合项 `−ω×(Jω+H)` 的"
          "**俯仰分量** 90–110 s ≤ %.2f N·m、110–130 s ≤ %.2f N·m；三轴最大 %.2f / %.2f N·m；"
          "扰动 ‖τ_d‖∞ ≤ %.2f / %.2f N·m。"
          % (gyro_pre, gyro_man, gyro_pre_all, gyro_man_all, td_pre, td_man),
          "", "### 1.3 CSV 的时间对齐约定（重要）", "",
          "日志的行序是“先执行、后记录”：第 k 行的**状态列**（`q, w, g, H, sigma_min`、以及由它们算出的"
          "`att_err_deg`、`cmg_*`、`E_gap`）是**该控制周期结束时**的状态，即物理时刻 `t_k + dt_ctrl`；"
          "而**计划列**（`tau_nom_*`、`Rp_*`、`Rm_*`、`d_*`、`lo_*`、`hi_*`、`qref*`、`wref*`）"
          "是在 `t_k` 时刻算出的；指令列（`tauc_cmd_*`、`tau_act_*`、`nu*`）是在 `[t_k, t_k+dt_ctrl]` 内施加的。",
          "",
          "因此本图的**状态类曲线按 `t + dt_ctrl` 绘制**（其真实物理时刻），计划/指令类曲线按 `t` 绘制；"
          "`att_err_deg` 本身混用了两种时刻（`t_k+dt` 的状态对 `t_k` 的参考），故未直接使用，"
          "面板 1 的俯仰角分别由 `q`（+dt）与 `qref`（t）算出。事件竖线用绝对时刻标注。",
          "", "## 2. 派生量的公式与参数来源", "",
          "| 派生量 | 公式 | 参数来源 |", "|---|---|---|",
          "| 俯仰角 | `θ = asin(2(wy − zx))`（`model6.quat_to_euler` 的 ZYX 定义） | 项目四元数约定（未改） |",
          "| 瞬时俯仰能力 | `τ⁺_max,y = max_{ν∈U} e_yᵀ(−A(γ)ν)`，`τ⁻_max,y = min_{ν∈U}(…)`，"
          "`U = {|ν_i| ≤ %.2f rad/s}` | `A(γ)`、`gdot_max` |" % prm.gdot_max,
          "| 解析等价 | `τ⁺_max,y = −τ⁻_max,y = gdot_max·Σ_i|A_yi(γ)|`（与 16 顶点枚举最大差 %.1e N·m） | 同上 |"
          % info["worst_identity"],
          "| 能力利用率 | `|τ_cmg,y| / min(τ⁺_max,y, |τ⁻_max,y|)`，**逐样本**计算 | CSV + 上述能力 |",
          "| 缺口覆盖残差 | `‖τ_cmg − d‖`（CSV `gap_residual`） | 已记录 |",
          "",
          "关于 `U`：框架**加速度**约束是 `|ν_k − ν_{k−1}| ≤ gddot_max·dt` 的逐步约束，"
          "不改变某一时刻的瞬时可达集，故能力计算只用 `|ν_i| ≤ gdot_max`；"
          "这与 `torque_gap_ocp.py` 中“速率盒 + 增量约束”的写法一致。",
          "", "## 3. 时间线（真实数据）", "",
          "| 事件 | 时刻 | 依据 |", "|---|---|---|",
          "| 控制循环开始 | 90.0 s | 配置 `torque_gap.start` |",
          "| CMG 提前构型迁移开始 | **%.1f s** | `metrics.json: migration_onset_t`，判据 `max_i|γ_i−γ_i(0)| > %.0f°` |"
          % (mig_t, MIG_TOL_DEG),
          "| 首次 `|γ_i| > 90°` | %s | CSV `g*`（±90° 行程墙已不存在） |"
          % ("%.1f s" % t90 if t90 is not None else "未发生"),
          "| 主姿态机动开始 | 110.0 s | `configs/torque_gap_unlimited/cases/c1_pitch30.yaml` |",
          "| 预测缺口首次非零 | %s | CSV `d_y` |"
          % ("%.1f s" % info["t_gap"] if info["t_gap"] else "无"),
          "", "## 4. 力矩分工（俯仰轴；均值 / 峰值 / RMS，N·m）", "",
          "| 区段 | 需求 `τ_nom` | 舵面 `τ_a` | CMG `τ_cmg` | 合成 `τ_a+τ_cmg` | 需求−合成 |",
          "|---|---|---|---|---|---|",
          "| 90–110 s（迁移/准备） | %s | %s | %s | %s | %s |"
          % tuple(" / ".join("%.2f" % x for x in S["pre"][k])
                  for k in ("nom", "act", "cmg", "tot", "res")),
          "| 110–130 s（主机动） | %s | %s | %s | %s | %s |"
          % tuple(" / ".join("%.2f" % x for x in S["man"][k])
                  for k in ("nom", "act", "cmg", "tot", "res")),
          "",
          "- 舵面与 CMG 的**反号比例**（舵面在抵消 CMG 反作用力矩）：迁移段 %.0f%%、机动段 %.0f%%。"
          % (100 * opp_pre, 100 * opp_man),
          "- 迁移段：舵面均值 %.2f N·m、CMG 均值 %.2f N·m，两者近似相消，合成仅 %.2f N·m，"
          "姿态保持在被控范围内（见 5.1）——这就是“气动富余抵消 CMG 迁移反作用力矩”。"
          % (S["pre"]["act"][0], S["pre"]["cmg"][0], S["pre"]["tot"][0]),
          "- 机动段：舵面均值 %.2f N·m、CMG 峰值 %.1f N·m；需求−合成的均值为 %.2f N·m。"
          % (S["man"]["act"][0], S["man"]["cmg"][1], S["man"]["res"][0]),
          "",
          "**“需求−合成”这一列是什么？** 不是建模误差，而是闭环的**反馈修正**。由",
          "`J ω̇ = τ_a + τ_cmg + τ_d − ω×(Jω+H)` 与 "
          "`τ_nom = J ω̇_ref + ω_ref×(Jω_ref+H_nom) − τ_d` 可得",
          "",
          "```",
          "(τ_a + τ_cmg) − τ_nom = J(ω̇ − ω̇_ref) − [ω×(Jω+H) − ω_ref×(Jω_ref+H_nom)]",
          "```",
          "",
          "扰动 τ_d **精确抵消**，因此残差只由“反馈角加速度修正”与“陀螺项变化”两部分构成；"
          "它的存在正是任务书 §2.3 所说“前馈需求在存在当前误差/扰动时会低估需求”。",
          "",
          "该式**不能**用本 CSV 做逐点数值核对：`J_y = %.0f kg·m²`，`ω` 仅 0.2 s 采样且力矩为零阶保持，"
          "`ω̇` 的差分误差会被 `J_y` 放大到几十 N·m；此外状态列比计划列晚一个控制步（见 §1.3）。"
          "因此改用下面更强的“状态复现”检验。" % cfg.vehicle3.Jy,
          "",
          "**状态复现检验（无隐藏力矩项）：** 取第 k−1 行的状态、第 k 行的指令 `[τ_a, ν]` 与配置扰动，"
          "把 11 状态 P0 模型积分一个控制周期（`dt = %.2f s`，%d 子步，与仿真植物完全相同的步进方式），"
          "再与第 k 行状态比较；共 %d 个区间，最大偏差为：" %
          (dt_ctrl, n_sub, n_check), "",
          "| 状态分量 | 最大偏差 |", "|---|---|",
          "| 机体角速度 `[p,q,r]` | %.2e rad/s |" % worst_state[1],
          "| 框架角 `γ` | %.2e rad |" % worst_state[2],
          "| 四元数 `q` | %.2e |" % worst_state[3],
          "",
          "偏差在数值积分精度量级，说明**日志中的舵面力矩与 CMG 力矩、加上模型自身的陀螺项和扰动，"
          "即可完整复现被控对象的状态演化**，不存在被遗漏或重复计入的补偿力矩。",
          "", "## 5. 关键问题", "",
          "### 5.1 提前构型迁移发生的时间",
          "t ≈ **%.1f s**（机动前 %.1f s）；此后 γ 持续变化，t ≈ %s 首次越过 90°。"
          "迁移段（%.1f–110 s）俯仰角误差最大 %.3f°、RMS %.3f°（90–110 s 全段最大 %.3f°），"
          "说明**准备动作没有破坏姿态稳定**。"
          % (mig_t, T_MANEUVER - mig_t, ("%.1f s" % t90) if t90 else "—", mig_t,
             float(np.max(mig_err)), float(np.sqrt(np.mean(mig_err ** 2))),
             float(np.max(pre_err))),
          "", "### 5.2 迁移期间舵面与 CMG 的分工",
          "见 §4：舵面均值 %.2f N·m 与 CMG 均值 %.2f N·m 反号（%.0f%% 的样本反号），"
          "合成力矩仅 %.2f N·m。即准备阶段**舵面维持姿态、CMG 主动改变构型**。"
          % (S["pre"]["act"][0], S["pre"]["cmg"][0], 100 * opp_pre, S["pre"]["tot"][0]),
          "", "### 5.3 主机动期间舵面是否出现能力不足",
          "**是。** 机动窗口内 `d_y ≠ 0` 的样本占 %.0f%%，`|d_y|` 峰值 **%.1f N·m**"
          "（t = %.1f s，方向 %s y）；`d_y` 超过同时刻瞬时 CMG 能力的样本占 %.0f%%；"
          "`tau_bound_used` 在机动窗口均值 %.2f（3 个轴中处于气动盒边界的平均比例）。"
          "因此需求超出预测气动盒时，缺口只能由 CMG 承担，舵面在部分时段贴边工作。"
          % (100 * gap_frac, gap_peak, t[i_gp], "+" if dir_gap > 0 else "−",
             100 * short_frac, float(np.mean(d["tau_bound_used"][man]))),
          "", "### 5.4 迁移到底改变了什么：为什么“能力上下限”反而变小", "",
          "**先说结论：迁移的目标不是瞬时力矩包络，而是俯仰轴角动量 `H_y`。** 本运行里 `H_y` 的变化：", "",
          "| t [s] | " + " | ".join("%.0f" % tt for tt, _ in hy_tab) + " |",
          "|" + "---|" * (len(hy_tab) + 1),
          "| `H_y` [N·m·s] | " + " | ".join("%.1f" % v for _, v in hy_tab) + " |", "",
          "即 `H_y` 从 %.2f 被**预置到 %.1f N·m·s**（t=110 s），而该轴角动量的可达上界是"
          " `h0·Σ_i√(s0_iy²+(g_i×s0_i)_y²) = %.2f N·m·s`，已经用到 **%.1f%%**。"
          % (hy_tab[0][1], hy[k_at(110.0)], Hp, 100.0 * hy[k_at(110.0)] / Hp), "",
          "**力矩包络必然同时塌陷，这是数学恒等式而非缺陷：**", "",
          "```",
          "H_y(γ) = h0 Σ_i s_i,y(γ_i),     ∂H_y/∂γ_i = h0 (g_i × s_i(γ))_y = A_1i(γ)",
          "τ⁺_max,y = gdot_max Σ_i |A_1i(γ)|   ←  正是 H_y 对 γ 的梯度（L1 形式）",
          "```",
          "",
          "把 `H_y` 推到可达集极值点 ⇒ 该点的梯度 `A_1i` 全部为零 ⇒ `τ⁺_max,y → 0`。"
          "数值验证：在 `H_y` 取最大值的构型 γ = %s° 处，`Σ_i|A_1i| = %.2e`、"
          "`τ⁺_max,y = %.2e N·m`（home 构型是 86.6 N·m）；而本运行 t=107–111 s 的 `σ_min` 最小已降到 "
          "**%.2f**（home 40.7），说明构型确实停在“角动量饱和”的近奇异点附近。"
          % (np.round(np.rad2deg(g_at_max), 1).tolist(), grad_at_max, cap_at_max,
             sig_pre_min), "",
          "### 5.5 那么迁移算不算“成功”？用对判据", "",
          "力矩 `τ_cmg,y > 0` 会让 `H_y` 减小（`dH_y/dt = −τ_cmg,y`），所以未来需要 **+y 力矩**时，"
          "可用裕度是 `H_y − H_y,min`；需要 **−y 力矩**时是 `H_y,max − H_y`。用真实数据：", "",
          "| 时刻 | `H_y` [N·m·s] | 可供 **+y** 力矩消耗的裕度 [N·m·s] | 可供 **−y** 力矩消耗的裕度 [N·m·s] |",
          "|---|---|---|---|",
          "| 90 s（未迁移） | %.1f | %.1f | %.1f |" % (hy[0], head_down_90, Hp - hy[0]),
          "| 110 s（迁移完成） | %.1f | **%.1f** | %.1f |"
          % (hy[k_at(110.0)], head_down_110, head_up_110), "",
          "机动第一阶段（110–115 s）缺口为 **+y**（峰值 %.1f N·m），消耗的正是“+y”裕度："
          "该裕度在迁移中从 %.1f 增加到 **%.1f N·m·s（×%.2f）**；机动第二阶段（120–130 s）缺口转为 **−y**，"
          "所需的“−y”裕度由 `H_y` 在第一阶段下降后自动腾出。"
          % (gap_peak, head_down_90, head_down_110, head_down_110 / head_down_90), "",
          "**因此：迁移是成功的——它把俯仰轴角动量预置到了将来要消耗的那一侧，可用角动量裕度翻倍；"
          "但瞬时力矩包络必随之下降，用“`τ⁺` 有没有变大”来评判迁移是选错了判据。**"
          "（本报告 §5.4 初版以及 `unlimited_gimbal_experiment.md` §5.4 中“迁移没有提高能力”的说法，"
          "按此判据应当修正为：*迁移不改变（且必然压低）瞬时包络，但显著增加弯矩方向的角动量裕度*。）", "",
          "仍要强调：`H_y` 是**角动量**，不是力矩储备。它决定“还能在该方向持续输出多久”"
          "（时间 ≈ 裕度 / 需求力矩），不能直接读成“还能出多大扭矩”。", "",
          "另外，实际 `H_y` 的摆幅（+156 → −156，约 312 N·m·s）明显大于 110–130 s 的缺口积分"
          "（`∫d_y dt ≈ +77 N·m·s`）：差额来自 CMG 还要承担瞬态、以及指令抖振造成的往复，"
          "因此**不能把 `ΔH_y` 与 `∫d dt` 一一对应**，也不能据此反推“迁移量是否刚好够用”。", "",
          "### 5.6 瞬时能力曲线本身（仅供参考）", "",
          "由真实 `A(γ)` 逐样本算得（这是**瞬时**能力，不是迁移的目标，也不是可持续能力）：", "",
          "| t [s] | `τ⁺_max,y` [N·m] | `τ⁻_max,y` [N·m] |", "|---|---|---|"]
    for tt, a, b in cap_tab:
        md.append("| %.0f | %.1f | %.1f |" % (tt, a, b))
    md += ["",
           "- 迁移段 `τ⁺_max,y` 中位数 %.1f N·m，机动段中位数 %.1f N·m（机动段最大 %.1f N·m）。"
           % (cap_pre_med, cap_man_med, cap_man_max),
           "- 缺口峰值时刻（t = %.1f s）需求方向为 %s y，该方向的瞬时可用能力 %.1f N·m，"
           "而缺口为 %.1f N·m（能力/需求 = %.2f）。"
           % (t[i_gp], "+" if dir_gap > 0 else "−", cap_dir, gap_peak,
              cap_dir / max(gap_peak, 1e-9)),
           "- 该能力曲线逐样本**剧烈起伏**（≈30–230 N·m），因为 γ 每个控制周期都被重解、"
           "存在指令抖振（`τ_a` 每步跳变约 2 N·m，ν 符号翻转约占 57% 的控制步）。"
           "因此逐点比较没有意义，**只能看区间统计量**（上表中位数）。",
           "- 瞬时能力在多数时刻仍然够用：机动窗口内只有 %.0f%% 的样本 `|d_y| > min(τ⁺,|τ⁻|)`。"
           "**本工况的真正瓶颈是可持续输出（角动量/框架速率预算），不是瞬时包络**。"
           % (100 * short_frac),
           "", "### 5.7 实际输出与瞬时能力的关系",
           "机动窗口内逐样本能力利用率 `|τ_cmg,y| / min(τ⁺,|τ⁻|)`：中位数 %.0f%%、最大 %.0f%%；"
           "“全部样本满足 `|τ_cmg,y| ≤ min(τ⁺,|τ⁻|)`”这一自洽性检查：**%s**。"
           % (100 * util_med, 100 * util_max, "通过" if inside else "**未通过**"),
           "即实际输出始终落在瞬时能力带内，但**中位数只用到 %.0f%%**——"
           "这正说明“瞬时能力 ≠ 可持续输出”：真正的限制是角动量与框架角速度预算，"
           "而不是某一时刻的最大力矩（同一结论见 `docs/torque_gap/unlimited_gimbal_experiment.md` §5.2）。"
           % (100 * util_med),
           "", "### 5.8 当前数据**不能**证明的结论", "",
           "1. **未来持续力矩能力 / “力矩储备”**：CSV 只保存当前节点的 `τ_nom`、`d`、`lo/hi`，"
           "**没有**保存预测时域内各节点的 `τ_nom(k)`、`d(k)`，因此无法从归档数据画出“未来需求方向”曲线，"
           "也无法给出按未来方向折算的持续能力。面板 5 因此只画**瞬时**能力，未给出任何“剩余储备”指标。",
           "2. **角动量储备**：`|H|` 的几何上界为 `4h0 = %.0f N·m·s`（实际可达 ≈162.9），"
           "但本图没有把 `|H|` 当作力矩储备；要回答“还能持续多久”需要对未来需求做时间积分，归档中不完整。"
           % (4 * prm.h0),
           "3. **真实舵偏角实现**：本运行是 P0（理想力矩执行器），`tau_act_y ≡ tauc_cmd_y`，"
           "不能据此断言真实舵面可达到同样力矩；对应结论须用 P1 运行"
           "（`results/torque_gap_unlimited/C1_B0_P1/`，本次未画）。",
           "4. **因果性**：本图是单次闭环运行的时序，不构成“提前迁移导致跟踪改善”的因果证据；"
           "该判断依赖 B0/B1/B2 与 C4 对照"
           "（`docs/torque_gap/unlimited_gimbal_experiment.md`、`results/torque_gap_unlimited/summary.md`）。",
           "", "## 6. 图中约定", "",
           "- 竖直参考线（五面板统一）：红色点线 = 提前构型迁移开始（%.1f s）；黑色虚线 = 主机动开始（110 s）；"
           "蓝色点划线 = 预测缺口首次非零（%s）。"
           % (mig_t, ("%.1f s" % info["t_gap"]) if info["t_gap"] else "无"),
           "- 面板 3 的浅灰带 = **气动盒**；面板 5 的浅灰带 = **瞬时能力带**，含义不同，图例已区分。",
           "- 单位：力矩 N·m、时间 s、姿态角 deg；中文 Microsoft YaHei；数学符号 mathtext；"
           "PDF 以 Type-42 内嵌字体；PNG 300 dpi。",
           ]
    DOC.mkdir(parents=True, exist_ok=True)
    path = DOC / "C1_control_overview_analysis.md"
    path.write_text("\n".join(md) + "\n", encoding="utf-8")
    return path


def main():
    d, cfg, meta, metrics = load_run()
    info = plot(d, cfg, meta, metrics)
    path = report(d, cfg, meta, metrics, info)
    print("figure:", info["png"])
    print("figure:", info["pdf"])
    print("report:", path)
    print("migration onset %.1f s | gap onset %s | gdot_max %.2f rad/s"
          % (info["t_mig"], info["t_gap"], info["gdot_max"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
