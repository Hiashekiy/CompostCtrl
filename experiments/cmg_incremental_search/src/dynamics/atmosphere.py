"""大气表（指导实施文件 §3.2）：NRLMSIS-2.0 对数插值表的本地副本。

表文件 `data/atmosphere_msis2_f107150.csv` 与旧项目 `results/_cache/msis2_table_150.csv`
**逐字节相同**（SHA-256 255a5aed…），条件冻结为 f107=f107a=150, ap=4, lat=45°, lon=0°,
2024-01-01。它只是一张查表数据，不引入对旧代码的依赖。
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parents[2] / "data" / "atmosphere_msis2_f107150.csv"


class TableAtmosphere:
    def __init__(self, path: str | Path | None = None):
        self.path = Path(path) if path else DATA
        z, r = [], []
        with self.path.open(encoding="utf-8", newline="") as fh:
            for row in csv.DictReader(line for line in fh if not line.startswith("#")):
                z.append(float(row["h_m"]))
                r.append(float(row["rho_kg_m3"]))
        self.z = np.asarray(z)
        self.rho_tab = np.asarray(r)

    def rho(self, h_m: float) -> float:
        h = float(np.clip(h_m, self.z[0], self.z[-1]))
        i = int(np.clip(np.searchsorted(self.z, h) - 1, 0, len(self.z) - 2))
        w = (h - self.z[i]) / (self.z[i + 1] - self.z[i])
        return float(np.exp(np.log(self.rho_tab[i])
                            + w * (np.log(self.rho_tab[i + 1]) - np.log(self.rho_tab[i]))))

    def qinf(self, h_m: float, V: float) -> float:
        return 0.5 * self.rho(h_m) * V * V
