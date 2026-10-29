#!/usr/bin/env python
"""Stage-1 verification of the legacy physics/CMG reference model.

The new experiment (``experiments/cmg_incremental_search``) must not depend on
``nearspace/``; this harness only *audits* the archived reference so that the
new implementation can be written against verified conventions instead of
assumptions.  Every check re-derives a quantity independently and compares it
with the legacy implementation; nothing outside ``tmp/`` is written.

Run from the repository root::

    python tools/verify_legacy_model.py            # human readable
    python tools/verify_legacy_model.py --json tmp/model_verification.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from nearspace.cmg import BETA_PYRAMID, CMGCluster, CMGParams          # noqa: E402
from nearspace.config import load_config                                # noqa: E402
from nearspace.model6 import (                                          # noqa: E402
    VehicleParams,
    cross_matrix,
    quat_mul,
    quat_normalize,
    quat_to_R,
)
from nearspace.model6c import (                                         # noqa: E402
    VehicleCMGParams,
    dynamics6c,
    limit_commands6c,
    make_cluster,
    plant_step6c,
)

RESULTS: list[dict] = []


def record(name: str, ok: bool, detail: str, value=None) -> None:
    RESULTS.append({"check": name, "ok": bool(ok), "detail": detail, "value": value})
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def rho_msis2(h_m: float) -> float:
    """Density at h from the archived NRLMSIS-2.0 table (read-only parse).

    The table is a git-tracked historical artefact (``results/_cache/``), so it is
    parsed directly instead of instantiating ``RealAtmosphere``: that class would
    write its cache, and ``tmp/`` is not writable in this sandbox.
    """
    import csv

    path = REPO / "results" / "_cache" / "msis2_table_150.csv"
    zs: list[float] = []
    rs: list[float] = []
    with path.open(encoding="utf-8", newline="") as fh:
        rows = (line for line in fh if not line.startswith("#"))
        for row in csv.DictReader(rows):
            zs.append(float(row["h_m"]))
            rs.append(float(row["rho_kg_m3"]))
    i = max(0, min(np.searchsorted(zs, h_m) - 1, len(zs) - 2))
    z0, z1, r0, r1 = zs[i], zs[i + 1], rs[i], rs[i + 1]
    w = (h_m - z0) / (z1 - z0)
    return float(np.exp(np.log(r0) + w * (np.log(r1) - np.log(r0))))


# ---------------------------------------------------------------- CMG checks
def check_geometry(cluster: CMGCluster) -> None:
    g = cluster.g_axis
    norms = np.linalg.norm(g, axis=0)
    dots = np.array([[abs(g[:, i] @ g[:, j]) for j in range(4)] for i in range(4)])
    off = dots[~np.eye(4, dtype=bool)]
    record(
        "C1 gimbal axis geometry",
        np.allclose(norms, 1.0) and np.allclose(off, 1.0 / 3.0, atol=1e-9),
        f"|g_i|=1, |g_i.g_j|={off.min():.6f}..{off.max():.6f} (expect 1/3), "
        f"beta={np.rad2deg(cluster.prm.beta):.4f} deg",
        float(off.mean()),
    )
    s = cluster.spin_axes(np.zeros(4))
    orth = np.array([abs(g[:, i] @ s[:, i]) for i in range(4)])
    record(
        "C2 spin axes orthogonal to gimbals",
        np.allclose(orth, 0.0, atol=1e-12),
        f"max |s_i.g_i| = {orth.max():.3e}",
        float(orth.max()),
    )


def check_jacobian(cluster: CMGCluster) -> None:
    rng = np.random.default_rng(20240101)
    err = 0.0
    for _ in range(20):
        gamma = rng.uniform(-np.pi, np.pi, 4)
        A = cluster.jacobian(gamma)
        fd = np.zeros((3, 4))
        for i in range(4):
            h = 1e-7
            gp = gamma.copy(); gp[i] += h
            gm = gamma.copy(); gm[i] -= h
            fd[:, i] = (cluster.momentum(gp) - cluster.momentum(gm)) / (2 * h)
        err = max(err, float(np.max(np.abs(A - fd))))
    record(
        "C3 A(gamma) == dh/dgamma",
        err < 1e-5,
        f"max |A - finite difference| = {err:.3e} over 20 random configurations",
        err,
    )


def check_sigma_home(cluster: CMGCluster) -> None:
    A0 = cluster.jacobian(np.zeros(4))
    sig = cluster.sigma_min(A0)
    claim = cluster.prm.home_sigma_min
    h0 = cluster.prm.h0
    exact = CMGCluster(CMGParams(h0=h0, beta=BETA_PYRAMID))
    sig_exact = exact.sigma_min(exact.jacobian(np.zeros(4)))
    guaranteed = sig * cluster.prm.gdot_max
    record(
        "C4 home singularity margin",
        abs(sig_exact - h0 * np.sqrt(2.0 / 3.0)) < 1e-9
        and abs(sig - claim) / sig < 1e-6,
        f"exact beta=arcsin(sqrt(2/3)): sigma_min(A(0)) = {sig_exact:.9f} = h0*sqrt(2/3); "
        f"configured beta={np.rad2deg(cluster.prm.beta):.6f} deg gives {sig:.9f} "
        f"(rel. dev. {(sig - claim) / sig:+.2e} from the hard-coded constant "
        f"home_sigma_min={claim:.9f}); floor={cluster.prm.sigma_floor:.4f}; "
        f"guaranteed torque=sigma_min*gdot_max={guaranteed:.2f} N.m",
        sig,
    )
    sv = np.linalg.svd(A0, compute_uv=False)
    det = cluster.det_AAt(A0)
    record(
        "C5 singularity indices agree",
        abs(det - float(np.prod(sv ** 2))) / abs(det) < 1e-12,
        f"det(A A^T) = {det:.6e} equals prod(sigma_i^2) = {float(np.prod(sv ** 2)):.6e}",
        det,
    )


def check_zero_rate(cluster: CMGCluster) -> None:
    gamma = np.array([0.3, -0.7, 1.1, -0.2])
    M = cluster.torque(gamma, np.zeros(4))
    record(
        "C6 zero gimbal rate -> zero CMG torque",
        np.allclose(M, 0.0),
        f"|M(gamma, gamma_dot=0)| = {np.linalg.norm(M):.3e} N.m",
        float(np.linalg.norm(M)),
    )


def check_capability(cluster: CMGCluster) -> None:
    A = cluster.jacobian(np.zeros(4))
    best = cluster.torque_capability(A / cluster.prm.h0) * cluster.prm.h0 * cluster.prm.gdot_max
    record(
        "C7 peak deliverable torque at home",
        best > 0,
        f"max ||M|| over |gamma_dot_i| <= {cluster.prm.gdot_max:g} rad/s = {best:.2f} N.m "
        f"(guaranteed, worst direction, is {cluster.prm.home_sigma_min * cluster.prm.gdot_max:.2f})",
        float(best),
    )


# ------------------------------------------------------------- plant checks
def params(cfg) -> tuple[VehicleCMGParams, CMGCluster]:
    prm = VehicleCMGParams.from_config(cfg)
    prm.veh = prm.veh.copy(Md0=np.zeros(3), Md_amp=np.zeros(3))
    return prm, make_cluster(prm)


def initial_state(prm: VehicleCMGParams) -> np.ndarray:
    x = np.zeros(18)
    x[0] = 1.0
    x[4:7] = np.array([0.05, -0.03, 0.02])
    x[10:14] = np.array([0.10, -0.20, 0.30, -0.15])
    x[14:18] = np.array([0.30, -0.20, 0.15, -0.25])
    return x


def check_momentum_balance(prm: VehicleCMGParams, cluster: CMGCluster) -> None:
    """Torque-free, disturbance-free: R(q) (J w + h(gamma)) must be constant.

    This is the sign statement of M_cmg = -A gamma_dot.  A wrong sign would leave
    the total angular momentum drifting.
    """
    x = initial_state(prm)
    p = np.zeros(7)
    p[1] = 4000.0                      # V_ned, rho = 0 -> no aero
    u = np.concatenate([np.zeros(3), x[14:18]])
    H0 = quat_to_R(x[0:4]) @ (prm.J @ x[4:7] + cluster.momentum(x[10:14]))
    dt, steps = 0.002, 2500            # 5 s
    peak = 0.0
    for _ in range(steps):
        x, _ = plant_step6c(x, u, p, dt, prm, cluster)
        H = quat_to_R(x[0:4]) @ (prm.J @ x[4:7] + cluster.momentum(x[10:14]))
        peak = max(peak, float(np.linalg.norm(H - H0)))
    rel = peak / max(np.linalg.norm(H0), 1e-12)
    record(
        "P1 inertial angular momentum conserved (sign of -A gamma_dot)",
        rel < 1e-6,
        f"|H(t)-H(0)|max / |H(0)| = {rel:.3e} over 5 s, zero external torque",
        rel,
    )


def check_quaternion_kinematics(prm: VehicleCMGParams, cluster: CMGCluster) -> None:
    # (a) convention: q_dot = 0.5 q (x) [0, omega] integrated with a FIXED omega
    #     must give R(t) = R(0) expm([omega]x t).
    rng = np.random.default_rng(7)
    q = quat_normalize(rng.normal(size=4))
    omega = np.array([0.12, -0.07, 0.05])
    R0 = quat_to_R(q)
    dt, T = 1e-4, 1.0
    for _ in range(int(T / dt)):
        q = quat_normalize(q + dt * 0.5 * quat_mul(q, np.array([0.0, *omega])))
    err_a = float(np.max(np.abs(quat_to_R(q) - R0 @ _expm_so3(omega, T))))
    record(
        "P2a quaternion kinematics convention (R_dot = R [omega]x)",
        err_a < 1e-10,
        f"max |R_integrated - R0 expm([omega]x t)| = {err_a:.2e} over 1 s, fixed omega",
        err_a,
    )

    # (b) the plant's own attitude kinematics, with a time-varying omega
    x = initial_state(prm)
    p = np.zeros(7)
    p[1] = 4000.0
    dt = 1e-4
    peak = 0.0
    for _ in range(500):
        R_before = quat_to_R(x[0:4])
        omega = x[4:7].copy()
        x, _ = plant_step6c(x, np.concatenate([np.zeros(3), x[14:18]]), p, dt, prm, cluster)
        D = quat_to_R(x[0:4]) @ R_before.T
        peak = max(peak, float(np.max(np.abs(D - _expm_so3(omega, dt)))))
    record(
        "P2b plant attitude kinematics",
        peak < 1e-9,
        f"max |R(t+dt) R(t)^T - expm([omega]x dt)| = {peak:.2e} over 50 ms of closed-form rotation",
        peak,
    )


def _expm_so3(omega: np.ndarray, t: float) -> np.ndarray:
    th = float(np.linalg.norm(omega)) * t
    K = cross_matrix(omega / max(np.linalg.norm(omega), 1e-15))
    if th < 1e-12:
        return np.eye(3) + K * th
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def check_servo_and_gimbal_limits(prm: VehicleCMGParams, cluster: CMGCluster) -> None:
    veh = prm.veh
    x = np.zeros(18)
    x[0] = 1.0
    u = np.concatenate([10.0 * np.ones(3), np.zeros(4)])   # saturated surface command
    p = np.zeros(7)
    p[1] = 4000.0
    peak_rate = 0.0
    for _ in range(200):
        x_new, u_app = plant_step6c(x, u, p, 0.005, prm, cluster)
        peak_rate = max(peak_rate, float(np.max(np.abs(x_new[7:10] - x[7:10]) / 0.005)))
        x = x_new
    ok = peak_rate <= float(np.max(veh.delta_dot_max)) + 1e-6
    record(
        "P3 surface deflection and rate limits",
        ok and np.all(np.abs(x[7:10]) <= veh.delta_max + 1e-9),
        f"max |delta_dot| = {peak_rate:.3f} rad/s, limit {np.max(veh.delta_dot_max):.3f}; "
        f"|delta|max = {np.max(np.abs(x[7:10])):.4f} rad, limit {np.max(veh.delta_max):.4f}",
        peak_rate,
    )

    x = np.zeros(18)
    x[0] = 1.0
    u = np.concatenate([np.zeros(3), 10.0 * np.ones(4)])
    peak_gdot = 0.0
    peak_gddot = 0.0
    for _ in range(400):
        x_new, _ = plant_step6c(x, u, p, 0.005, prm, cluster)
        peak_gdot = max(peak_gdot, float(np.max(np.abs(x_new[14:18]))))
        peak_gddot = max(peak_gddot, float(np.max(np.abs(x_new[14:18] - x[14:18]) / 0.005)))
        x = x_new
    rate_ok = peak_gdot <= cluster.prm.gdot_max + 1e-6
    record(
        "P4 gimbal rate limit enforced",
        rate_ok,
        f"max |gamma_dot| = {peak_gdot:.4f} rad/s, limit {cluster.prm.gdot_max:g}",
        peak_gdot,
    )
    record(
        "P5 gimbal acceleration limit NOT enforced (reported gap)",
        peak_gddot > cluster.prm.gddot_max,
        f"max |gamma_ddot| measured = {peak_gddot:.2f} rad/s^2 vs declared gddot_max="
        f"{cluster.prm.gddot_max:g}; plant_step6c limits only the rate, the first-order "
        f"gimbal loop (tau_g={cluster.prm.tau_g:g} s) does not bound acceleration",
        peak_gddot,
    )
    record(
        "P6 gimbal mechanical travel is NOT modelled (reported gap)",
        not any("travel" in f or "gamma_max" in f or "range" in f for f in CMGParams.__dataclass_fields__),
        "CMGParams has no travel/range field: gimbal angles are unbounded and never wrapped; "
        "the new experiment must take the real travel range from a hardware source",
    )


def check_aero_authority(cfg) -> None:
    veh = VehicleParams.from_config(cfg)
    h = 100_000.0
    rho = rho_msis2(h)
    V = 4000.0
    q_inf = 0.5 * rho * V * V
    delta = veh.delta_max
    M = q_inf * veh.S * np.array([veh.b * veh.Cl_da * delta[1],
                                  veh.cbar * veh.Cm_de * delta[0],
                                  veh.b * veh.Cn_dr * delta[2]])
    record(
        "P7 aerodynamic authority at 100 km",
        True,
        f"rho(100 km, MSIS2 f107=150) = {rho:.3e} kg/m^3, q_inf(V={V:g} m/s) = {q_inf:.2f} Pa, "
        f"|M_aero| at full deflection = {np.linalg.norm(M):.2f} N.m "
        f"(pitch {M[1]:.2f}, roll {M[0]:.2f}, yaw {M[2]:.2f})",
        float(np.linalg.norm(M)),
    )
    return q_inf


def report(prm: VehicleCMGParams, cluster: CMGCluster) -> None:
    v = prm.veh
    c = prm.cmg
    print("\n--- parameter summary (from config.yaml) ---")
    print(f"inertia J = diag({v.Jx:g}, {v.Jy:g}, {v.Jz:g}) kg m^2, mass {v.mass:g} kg")
    print(f"aero S={v.S:g} m^2, b={v.b:g} m, cbar={v.cbar:g} m")
    print(f"surface: delta_max={np.rad2deg(v.delta_max)} deg, "
          f"rate_max={np.rad2deg(v.delta_dot_max)} deg/s, servo tau_s={v.tau_s:g} s")
    print(f"CMG: n={c.n}, beta={np.rad2deg(c.beta):.4f} deg, h0={c.h0:g} N.m.s/unit "
          f"(cluster {c.h_total:g} N.m.s)")
    print(f"     gdot_max={c.gdot_max:g} rad/s, gddot_max={c.gddot_max:g} rad/s^2, "
          f"tau_g={c.tau_g:g} s, rotor {c.rotor_rpm:g} rpm "
          f"(Jr={c.h0 / (c.rotor_rpm * 2 * np.pi / 60):.4f} kg m^2)")
    print(f"     sigma_floor={c.sigma_floor:.4f} N.m.s "
          f"({c.sigma_floor_frac:g} of home {c.home_sigma_min:.4f})")
    print(f"     peak deliverable torque = {c.torque_per_unit:.2f} N.m/unit, "
          f"cluster guaranteed = {c.home_sigma_min * c.gdot_max:.2f} N.m")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", default=None, help="write the machine-readable summary here")
    args = parser.parse_args()

    cfg = load_config(str(REPO / "config.yaml"))
    prm, cluster = params(cfg)
    report(prm, cluster)

    check_geometry(cluster)
    check_jacobian(cluster)
    check_sigma_home(cluster)
    check_zero_rate(cluster)
    check_capability(cluster)
    check_momentum_balance(prm, cluster)
    check_quaternion_kinematics(prm, cluster)
    check_servo_and_gimbal_limits(prm, cluster)
    check_aero_authority(cfg)

    failed = [r for r in RESULTS if not r["ok"]]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks pass; "
          f"{len(failed)} reported as gaps/limitations")
    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"checks": RESULTS}, ensure_ascii=False, indent=2) + "\n",
                       encoding="utf-8", newline="\n")
        print(f"json -> {out}")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
