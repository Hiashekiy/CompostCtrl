"""时序日志（指导实施文件 §3.5/§10）：三路力矩必须分开记录。

字段分三组：
    * 期望：tau_req（控制器输出的期望总力矩）
    * 预测：tau_pred（分配器按同一模型预测的、本轮指令可实现的总力矩）
    * 真实：tau_actual / M_aero_actual / M_cmg_actual（plant 在该状态下实际产生的力矩）
另有舵面、框架、奇异性、残差、耗时与回退标记。禁止在图上手改数据，一切图由本日志生成。
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

VECTOR_FIELDS = (
    "tau_req", "tau_pred", "tau_actual", "M_aero_actual", "M_cmg_actual",
    "delta_cmd", "delta", "delta_dot", "gamma", "gdot", "omega", "q", "q_ref",
    "H_cmg", "residual",
)
SCALAR_FIELDS = (
    "t", "sigma_min", "H_norm", "err_norm_deg", "err_roll_deg", "err_pitch_deg",
    "err_yaw_deg", "tau_req_norm", "tau_pred_norm", "tau_actual_norm",
    "residual_norm", "rho", "qinf", "V", "h_km", "aero_sat_frac", "gdot_sat_frac",
    "feasible", "fallback", "solve_time_ms", "ctrl_time_ms", "search_time_ms",
    "search_nodes", "search_status", "gdot_ref_norm", "gamma_travel_frac",
)


class TimeSeriesLogger:
    def __init__(self):
        self.rows: list[dict] = []

    def log(self, t: float, **fields) -> None:
        row = {"t": float(t)}
        row.update(fields)
        self.rows.append(row)

    def __len__(self) -> int:
        return len(self.rows)

    def arrays(self) -> dict[str, np.ndarray]:
        out: dict[str, np.ndarray] = {}
        if not self.rows:
            return out
        keys = set().union(*[set(r) for r in self.rows])
        for k in sorted(keys):
            vals = [r.get(k) for r in self.rows]
            if any(isinstance(v, np.ndarray) for v in vals):
                out[k] = np.array([np.atleast_1d(v) if v is not None else np.full(1, np.nan)
                                   for v in vals], dtype=float).squeeze()
            elif any(isinstance(v, bool) for v in vals):
                out[k] = np.array([bool(v) if v is not None else False for v in vals])
            else:
                out[k] = np.array([np.nan if v is None else v for v in vals])
        return out

    def save(self, outdir: str | Path, stem: str = "timeseries") -> dict[str, str]:
        outdir = Path(outdir)
        outdir.mkdir(parents=True, exist_ok=True)
        arr = self.arrays()
        npz_path = outdir / f"{stem}.npz"
        np.savez_compressed(npz_path, **arr)
        csv_path = outdir / f"{stem}.csv"
        scalar_cols = [k for k in SCALAR_FIELDS if k in arr]
        vec_cols = [k for k in VECTOR_FIELDS if k in arr]
        header = list(scalar_cols)
        for k in vec_cols:
            width = arr[k].shape[1] if arr[k].ndim > 1 else 1
            header += [f"{k}_{i}" for i in range(width)]
        with csv_path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(header)
            for i in range(len(self.rows)):
                row = [arr[k][i] if arr[k].ndim == 1 else "" for k in scalar_cols]
                for k in vec_cols:
                    row += list(np.atleast_1d(arr[k][i]))
                writer.writerow(row)
        return {"npz": str(npz_path), "csv": str(csv_path)}


def save_json(obj, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, default=_jsonable) + "\n",
                    encoding="utf-8", newline="\n")


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    return str(o)
