"""Three-axis controllers."""
from __future__ import annotations

import numpy as np

from nearspace.model6 import VehicleParams
from legacy.mpc3 import LTVMPC3


class UnifiedMPC3Controller:
    name = "T3 unified MPC"

    def __init__(self, prm: VehicleParams, cfg):
        self.prm = prm
        self.cfg = cfg
        self.mpc = LTVMPC3(prm, cfg)

    def act(self, x, t, traj):
        return self.mpc.solve(x, t, traj)
