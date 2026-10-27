"""Sparse QP solver used by every MPC in this project.

    min 0.5 z'Hz + g'z   s.t.  l <= A z <= u

Row-scaled OSQP (the project normalises the rows itself, so OSQP internal
scaling must stay OFF) with an ADMM fallback.  The scaling is essential: the
momentum rows carry entries of order 50 while the gimbal-rate rows carry
entries of order 1, and OSQP diverges on the mixture otherwise.
"""
from __future__ import annotations

import numpy as np
from scipy.linalg import cho_factor, cho_solve

try:  # optional dependency
    import osqp as _osqp
    import scipy.sparse as _sp
    _HAVE_OSQP = True
except Exception:  # pragma: no cover
    _HAVE_OSQP = False


class ADMMFallback:
    def __init__(self, rho: float = 1.0, sigma: float = 1e-5,
                 max_iter: int = 20000, tol: float = 1e-6, alpha: float = 1.6):
        self.rho = rho
        self.sigma = sigma
        self.max_iter = max_iter
        self.tol = tol
        self.alpha = alpha

    def solve(self, H, g, A, l, u, z0=None):
        nz = H.shape[0]
        P = H + self.sigma * np.eye(nz) + self.rho * (A.T @ A)
        P = 0.5 * (P + P.T)
        try:
            factor = cho_factor(P, lower=True, check_finite=False)
        except np.linalg.LinAlgError:
            factor = cho_factor(P + 1e-6 * np.eye(nz), lower=True, check_finite=False)
        At = A.T
        z = np.zeros(nz) if z0 is None else np.array(z0, dtype=float).copy()
        y = np.zeros(A.shape[0])
        v = np.clip(A @ z, l, u)
        for _ in range(self.max_iter):
            z = cho_solve(factor, self.sigma * z - g + self.rho * (At @ (v - y)),
                          check_finite=False)
            Az = A @ z
            v_new = np.clip(self.alpha * Az + (1.0 - self.alpha) * v + y, l, u)
            y = y + self.alpha * Az + (1.0 - self.alpha) * v - v_new
            v = v_new
        return z


class QPSolver:
    """Row-scaled OSQP (fast) with ADMM fallback; stable call signature."""

    def __init__(self, max_iter: int = 20000, eps_abs: float = 1e-7,
                 eps_rel: float = 1e-7, use_osqp: bool = True,
                 internal_scaling: bool = True):
        self.max_iter = max_iter
        self.eps_abs = eps_abs
        self.eps_rel = eps_rel
        self.internal_scaling = bool(internal_scaling)
        self.use_osqp = bool(use_osqp and _HAVE_OSQP)
        self.fallback = ADMMFallback()
        self.last_status = "not_solved"
        self.last_iter = 0

    def solve(self, H, g, A, l, u, z0=None):
        scale = np.max(np.abs(A), axis=1)
        scale = np.where(scale > 0.0, scale, 1.0)
        As = A / scale[:, None]
        ls = np.where(np.isfinite(l), l, -1e30) / scale
        us = np.where(np.isfinite(u), u, 1e30) / scale
        ls = np.clip(ls, -1e30, 1e30)
        us = np.clip(us, -1e30, 1e30)
        if self.use_osqp and np.any(ls > us + 1e-9):  # invalid bounds -> fallback
            self.use_osqp = False
        if self.use_osqp:
            try:
                P = _sp.triu(_sp.csc_matrix(H)).tocsc()
                prob = _osqp.OSQP()
                prob.setup(P=P, q=np.asarray(g, dtype=float), A=_sp.csc_matrix(As),
                           l=np.asarray(ls, dtype=float), u=np.asarray(us, dtype=float),
                           verbose=False, eps_abs=self.eps_abs, eps_rel=self.eps_rel,
                           max_iter=self.max_iter, polish=True, warm_starting=True,
                           scaling=(1 if self.internal_scaling else 0))
                if z0 is not None:
                    prob.warm_start(x=np.asarray(z0, dtype=float), y=np.zeros(A.shape[0]))
                res = prob.solve()
                self.last_status = str(res.info.status)
                self.last_iter = int(res.info.iter)
                if res.x is not None and np.all(np.isfinite(res.x)):
                    return np.asarray(res.x, dtype=float)
            except Exception as exc:  # pragma: no cover
                self.last_status = "osqp_failed:%s" % exc
        self.last_status = "admm_fallback"
        return self.fallback.solve(H, g, A, l, u, z0=z0)
