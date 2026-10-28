# C1 reproduction verification

| item | value |
|---|---|
| git_commit | `c38310255f600110b4d7b677a829082964eda946` |
| python | `3.14.7` |
| platform | `Windows-10-10.0.19045-SP0` |
| executable | `C:\Python314\python.exe` |
| packages | {"numpy": "2.5.3", "scipy": "1.18.1", "osqp": "1.1.3", "pyyaml": "6.0.3", "matplotlib": "3.11.2", "pymsis": "0.13.0"} |

## metrics (committed history vs. re-run in this environment)

| metric | historical | re-run | relative diff |
|---|---|---|---|
| rmse_deg | 10.413920 | 10.413920 | +0.000 % |
| max_err_deg | 31.585378 | 31.585378 | +0.000 % |
| sat_fraction | 0.000000 | 0.000000 | +nan % |
| n_qp_fail | 0.000000 | 0.000000 | +nan % |
| eta_min | -895.937169 | -895.937169 | +0.000 % |

## per-step timeline difference (re-run minus historical)

| column | max abs | rms |
|---|---|---|
| theta | 0.000e+00 | 0.000e+00 |
| q | 0.000e+00 | 0.000e+00 |
| delta | 0.000e+00 | 0.000e+00 |
| theta_ref | 0.000e+00 | 0.000e+00 |
| err_deg | 0.000e+00 | 0.000e+00 |

## acceptance checks (fixed before the run)

| check | result | value | target | note |
|---|---|---|---|---|
| rmse_vs_history | PASS | 10.4139 | 10.4139 | re-run RMSE within 2% of the committed C1 RMSE |
| rmse_vs_quoted | PASS | 10.4139 | 10.4139 | re-run RMSE within 2% of the 10.41 deg quoted in the plan |
| peak_vs_history | PASS | 31.5854 | 31.5854 | peak error within 2% of the committed value |
| peak_vs_quoted | PASS | 31.5854 | 31.5854 | peak error within 2% of the 31.6 deg quoted in the plan |
| not_diverged | PASS | 10.4139 | 90 | a diverged record is never a reproduction |
| qp_fail_same | PASS | 0 | 0 | QP failure count identical |
| sat_fraction_same | PASS | 0 | 0 | saturation fraction within 1 pp |
| timeline_rows | PASS | 1000 | 1000 | same number of recorded steps |
| timeline_identical_to_1e-9 | PASS | 0 | 1e-09 | per-step states reproduce bit-for-bit if the environment is unchanged |

Verdict: **PASSED**

The 10.41 deg / 31.6 deg pair quoted in the plan is the historical record (10.4139 / 31.5854 in `legacy/results/metrics.csv`); the numbers above are what this environment actually produced. The older 10.55 deg figure that circulates in the project notes is *not* reproduced here and is not treated as evidence of a library-version cause.
