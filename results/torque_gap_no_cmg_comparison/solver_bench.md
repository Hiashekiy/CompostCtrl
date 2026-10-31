# Solver microbenchmark — no-CMG experiment MPC

Raw log: `solver_bench.log` (produced by `scripts/bench_no_cmg_solver.py`).
One and the same NLP: S00, B group, 22 nodes, 407 variables / 364 constraints,
state at t = 106 s with a 1 deg pitch offset (torque gap active).

## Valid rows (cold = no warm start, warm = previous solution shifted)

| setting | cold [s] | cold iters | cold objective | warm [s] | warm iters | warm objective |
|---|---:|---:|---:|---:|---:|---:|
| ipopt exact+mumps tol 1e-6 (current) | 1.522 | 279 | +10.078 | 0.572 | 156 | +4.346 |
| ipopt exact+mumps tol 1e-4 | 1.528 | 279 | +10.078 | 0.503 | 135 | +4.346 |
| **ipopt exact+mumps tol 1e-3** | 1.541 | 279 | +10.078 | **0.310** | 84 | +4.362 |
| ipopt exact+mumps tol 1e-2 | 1.561 | 279 | +10.078 | 0.275 | 74 | +4.368 |

(The cold solves stop at the same point regardless of 	ol: from a cold start
IPOPT terminates the same way and lands at a *worse* local point, objective
+10.08 versus +4.35 from the warm start.  Warm starting is therefore worth more
than any solver option here: 2.7x in time and a materially better solution.)

## Rows that must not be used as evidence

| setting | outcome | interpretation |
|---|---|---|
| ipopt lim-mem tol 1e-3 | 600 iterations, feasible-violation 9e-5, not converged | BFGS/LM Hessian is unusable for this problem (matches the note already in SolverConfig) |
| ipopt ma27 / ma57 | Invalid_Option | those linear solvers are not compiled into the pip IPOPT build on this host |
| ipopt jit | IpoptInterface::finalize error | no C compiler available to CasADi on this host |
| SQP gauss-newton + qpoases (1/3/20 it) | qpOASES: maximum number of working set recalculations | the per-iteration QP (407 vars, dense Lagrangian Hessian) is too hard for qpOASES with default settings; needs a different QP solver |
| SQP + hpipm | plugin not linked | hpipm is not part of this CasADi build |

## Why the per-iteration cost is high

From the sqpmethod construction print (same graph):

- variables 407, constraints 364
- constraint Jacobian: **3 201** non-zeros (sparse, block-banded as expected)
- Lagrangian Hessian: **165 649** non-zeros = 407^2, i.e. **fully dense**

A dense Lagrangian Hessian is what makes one IPOPT iteration cost ~8 ms
(measured 6.7–9.6 ms/iteration over the 8 P0 B-group runs, 35–145 iterations per
control step).  Reducing iterations (tolerance, warm start) and reducing the
problem size are therefore the only cheap levers; the structural fix is a
stage-wise formulation with a Gauss-Newton Hessian, which CasADi/IPOPT cannot
express here.
