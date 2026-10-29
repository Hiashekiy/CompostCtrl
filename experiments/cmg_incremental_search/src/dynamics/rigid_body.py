"""三轴刚体与姿态数学（指导实施文件 §3.1）。

约定（与阶段 1 核验过的参考实现一致，见 docs/模型核验-CMG构型路径搜索基准.md）：
    * 惯性系 NED：x 北、y 东、z 下。
    * 机体系：x 前、y 右、z 下。
    * 四元数 q = [w, x, y, z]（标量在前，Hamilton），把机体向量转到惯性系：v_n = R(q) v_b。
    * R_dot = R [omega]x,  omega = [p, q, r] 机体系 rad/s。
    * 姿态误差用旋转矢量 2 * dq_vec（dq = q_ref^-1 ⊗ q，取正半球），大角度不使用欧拉角差。

刚体角动量方程（含 CMG 角动量）：

    J w_dot = -w x (J w + H_cmg) + tau_aero + tau_cmg + tau_dist ,  tau_cmg = -A(gamma) gamma_dot
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_to_R(q):
    """机体 -> 惯性系的旋转矩阵。"""
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_quat(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) 欧拉角 -> q_bn。"""
    cr, sr = np.cos(0.5 * roll), np.sin(0.5 * roll)
    cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
    cy, sy = np.cos(0.5 * yaw), np.sin(0.5 * yaw)
    return quat_normalize(np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]))


def quat_to_euler(q):
    """ZYX (yaw-pitch-roll)，仅用于日志与作图，不用于控制误差。"""
    w, x, y, z = quat_normalize(q)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_error(q_ref, q):
    """旋转矢量误差（rad，机体系，q_ref 为目标）。"""
    dq = quat_mul(quat_conj(q_ref), q)
    if dq[0] < 0.0:
        dq = -dq
    return 2.0 * dq[1:4]


def quat_geodesic(qa, qb):
    """两个姿态之间的测地角（rad）。"""
    d = float(np.dot(quat_normalize(qa), quat_normalize(qb)))
    return 2.0 * np.arccos(min(1.0, abs(d)))


def cross_matrix(v):
    v = np.asarray(v, dtype=float)
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


@dataclass
class Inertia:
    """主轴惯量矩阵 J = diag(Jx, Jy, Jz)，kg m^2。"""

    Jx: float = 1200.0
    Jy: float = 8500.0
    Jz: float = 9000.0

    @property
    def J(self) -> np.ndarray:
        return np.diag([self.Jx, self.Jy, self.Jz])

    @property
    def Jinv(self) -> np.ndarray:
        return np.diag([1.0 / self.Jx, 1.0 / self.Jy, 1.0 / self.Jz])
