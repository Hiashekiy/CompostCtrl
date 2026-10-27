"""Piecewise-exponential rarefied atmosphere (20-100 km focus).

rho(h) = rho_ref * exp(-(h - h_lo)/H)  per layer, fitted to US Standard 1976.
Also provides temperature, speed of sound, pressure, mean free path and the
Knudsen number used for the rarefaction discussion.
"""
from __future__ import annotations

import numpy as np

K_B = 1.380649e-23
R_AIR = 287.0
GAMMA_AIR = 1.4


class Atmosphere:
    def __init__(self, layers):
        self.layers = list(layers)
        self.h_min = float(self.layers[0].h_lo)
        self.h_max = float(self.layers[-1].h_hi)

    def _layer(self, h: float):
        h = float(np.clip(h, self.h_min, self.h_max))
        for lay in self.layers:
            if h <= lay.h_hi:
                return lay
        return self.layers[-1]

    def rho(self, h: float) -> float:
        lay = self._layer(h)
        return float(lay.rho_ref * np.exp(-(float(h) - lay.h_lo) / lay.H))

    def T(self, h: float) -> float:
        lay = self._layer(h)
        return float(lay.T + lay.grad * (float(h) - lay.h_lo))

    def a(self, h: float) -> float:
        return float(np.sqrt(GAMMA_AIR * R_AIR * max(self.T(h), 100.0)))

    def p(self, h: float) -> float:
        return self.rho(h) * R_AIR * self.T(h)

    def mean_free_path(self, h: float, d_molecule: float = 3.7e-10) -> float:
        return K_B * self.T(h) / (np.sqrt(2.0) * np.pi * d_molecule ** 2 * self.p(h))

    def knudsen(self, h: float, cbar: float) -> float:
        return self.mean_free_path(h) / cbar
