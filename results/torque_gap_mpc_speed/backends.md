# MPC backend comparison (same experiment, three backends)

Each column set is the fully re-run 34-run A/B matrix with one MPC backend, so both arms, all scenarios and all seeds use the same solver.

| backend | phase | scenario/arm | seed | manoeuvre RMSE [deg] | mean [ms] | p95 [ms] | max [ms] | fallback | real-time |
|---|---|---|---|---|---|---|---|---|---|
| baseline | P0 | S00_pitch30/no_cmg | 0 | 4.6643 | 84.1 | 341.1 | 1236.7 | 0.000 | no |
| baseline | P0 | S00_pitch30/no_cmg | 1 | 4.6643 | 100.8 | 366.0 | 1808.8 | 0.000 | no |
| baseline | P0 | S00_pitch30/no_cmg | 2 | 4.6642 | 87.7 | 299.7 | 2353.1 | 0.003 | no |
| baseline | P0 | S00_pitch30/with_cmg | 0 | 0.0956 | 955.3 | 4297.8 | 8075.4 | 0.060 | no |
| baseline | P0 | S00_pitch30/with_cmg | 1 | 0.0954 | 829.8 | 2931.5 | 8121.8 | 0.050 | no |
| baseline | P0 | S00_pitch30/with_cmg | 2 | 0.0970 | 858.2 | 3658.9 | 7847.5 | 0.050 | no |
| baseline | P0 | S01_pitch_minus30/no_cmg | 0 | 5.0932 | 103.5 | 218.1 | 2281.4 | 0.007 | no |
| baseline | P0 | S01_pitch_minus30/no_cmg | 1 | 5.1163 | 112.4 | 228.5 | 2444.3 | 0.003 | no |
| baseline | P0 | S01_pitch_minus30/no_cmg | 2 | 5.0751 | 49.3 | 94.5 | 978.7 | 0.000 | yes |
| baseline | P0 | S01_pitch_minus30/with_cmg | 0 | 0.1519 | 686.4 | 1817.3 | 6151.1 | 0.007 | no |
| baseline | P0 | S01_pitch_minus30/with_cmg | 1 | 0.1317 | 673.4 | 2184.5 | 7466.3 | 0.030 | no |
| baseline | P0 | S01_pitch_minus30/with_cmg | 2 | 0.3017 | 752.3 | 2425.8 | 5358.4 | 0.017 | no |
| baseline | P0 | S02_pitch10/no_cmg | 0 | 0.5534 | 17.1 | 25.0 | 43.7 | 0.000 | yes |
| baseline | P0 | S02_pitch10/with_cmg | 0 | 0.0073 | 238.8 | 654.2 | 3421.5 | 0.000 | no |
| baseline | P0 | S03_pitch45/no_cmg | 0 | 10.9871 | 91.3 | 183.6 | 964.1 | 0.000 | yes |
| baseline | P0 | S03_pitch45/with_cmg | 0 | 2.2374 | 1393.6 | 7336.4 | 8387.7 | 0.117 | no |
| baseline | P0 | S04_roll30/no_cmg | 0 | 4.0012 | 72.3 | 194.6 | 536.7 | 0.000 | yes |
| baseline | P0 | S04_roll30/with_cmg | 0 | 0.0100 | 560.8 | 2300.8 | 7905.5 | 0.030 | no |
| baseline | P0 | S05_yaw30/no_cmg | 0 | 17.1372 | 157.8 | 278.3 | 2413.8 | 0.003 | no |
| baseline | P0 | S05_yaw30/with_cmg | 0 | 3.2047 | 628.1 | 1378.8 | 6472.2 | 0.007 | no |
| baseline | P0 | S06_pitch30_roll20/no_cmg | 0 | 4.9150 | 98.6 | 257.1 | 2669.9 | 0.003 | no |
| baseline | P0 | S06_pitch30_roll20/no_cmg | 1 | 4.9144 | 103.6 | 231.8 | 2201.9 | 0.003 | no |
| baseline | P0 | S06_pitch30_roll20/no_cmg | 2 | 4.9137 | 93.1 | 235.3 | 1338.8 | 0.000 | no |
| baseline | P0 | S06_pitch30_roll20/with_cmg | 0 | 0.0977 | 679.1 | 1557.5 | 7011.7 | 0.010 | no |
| baseline | P0 | S06_pitch30_roll20/with_cmg | 1 | 0.1161 | 704.6 | 1557.1 | 6206.1 | 0.020 | no |
| baseline | P0 | S06_pitch30_roll20/with_cmg | 2 | 0.0955 | 672.1 | 1710.2 | 6039.0 | 0.023 | no |
| baseline | P0 | S07_sequential/no_cmg | 0 | 2.7563 | 40.6 | 75.9 | 976.2 | 0.000 | yes |
| baseline | P0 | S07_sequential/with_cmg | 0 | 0.0629 | 1104.8 | 4439.0 | 8032.6 | 0.084 | no |
| baseline | P1 | S00_pitch30/no_cmg | 0 | 8.8693 | 116.7 | 329.1 | 2456.2 | 0.007 | no |
| baseline | P1 | S00_pitch30/with_cmg | 0 | 0.1687 | 926.3 | 4842.8 | 8193.8 | 0.057 | no |
| baseline | P1 | S01_pitch_minus30/no_cmg | 0 | 10.5442 | 131.2 | 311.2 | 2582.7 | 0.003 | no |
| baseline | P1 | S01_pitch_minus30/with_cmg | 0 | 0.1509 | 716.8 | 1739.8 | 6267.6 | 0.013 | no |
| baseline | P1 | S06_pitch30_roll20/no_cmg | 0 | 8.7005 | 83.8 | 196.2 | 968.0 | 0.000 | yes |
| baseline | P1 | S06_pitch30_roll20/with_cmg | 0 | 0.1385 | 853.1 | 2610.4 | 7479.1 | 0.053 | no |
| fast | P0 | S00_pitch30/no_cmg | 0 | 4.7457 | 42.6 | 126.5 | 391.9 | 0.000 | yes |
| fast | P0 | S00_pitch30/no_cmg | 1 | 4.6864 | 44.4 | 140.1 | 341.0 | 0.000 | yes |
| fast | P0 | S00_pitch30/no_cmg | 2 | 4.7483 | 43.4 | 133.6 | 352.7 | 0.000 | yes |
| fast | P0 | S00_pitch30/with_cmg | 0 | 0.0898 | 268.1 | 633.5 | 1200.8 | 0.007 | no |
| fast | P0 | S00_pitch30/with_cmg | 1 | 0.0983 | 263.3 | 505.7 | 1175.3 | 0.007 | no |
| fast | P0 | S00_pitch30/with_cmg | 2 | 0.1031 | 282.6 | 679.5 | 1238.7 | 0.007 | no |
| fast | P0 | S01_pitch_minus30/no_cmg | 0 | 5.0607 | 46.7 | 99.2 | 354.4 | 0.000 | yes |
| fast | P0 | S01_pitch_minus30/no_cmg | 1 | 5.0857 | 46.5 | 97.8 | 252.3 | 0.000 | yes |
| fast | P0 | S01_pitch_minus30/no_cmg | 2 | 5.0290 | 47.4 | 113.8 | 181.7 | 0.000 | yes |
| fast | P0 | S01_pitch_minus30/with_cmg | 0 | 0.1535 | 240.0 | 527.4 | 1167.4 | 0.000 | no |
| fast | P0 | S01_pitch_minus30/with_cmg | 1 | 0.1808 | 243.8 | 546.9 | 1149.1 | 0.000 | no |
| fast | P0 | S01_pitch_minus30/with_cmg | 2 | 0.1537 | 250.9 | 622.5 | 1270.9 | 0.007 | no |
| fast | P0 | S02_pitch10/no_cmg | 0 | 0.5532 | 13.8 | 20.5 | 32.6 | 0.000 | yes |
| fast | P0 | S02_pitch10/with_cmg | 0 | 0.0212 | 109.7 | 307.4 | 1194.0 | 0.003 | no |
| fast | P0 | S03_pitch45/no_cmg | 0 | 11.0194 | 67.8 | 176.6 | 391.7 | 0.010 | yes |
| fast | P0 | S03_pitch45/with_cmg | 0 | 2.1931 | 421.7 | 1178.8 | 1476.8 | 0.083 | no |
| fast | P0 | S04_roll30/no_cmg | 0 | 4.0034 | 37.1 | 83.0 | 106.0 | 0.000 | yes |
| fast | P0 | S04_roll30/with_cmg | 0 | 0.0144 | 138.6 | 456.9 | 1130.2 | 0.000 | no |
| fast | P0 | S05_yaw30/no_cmg | 0 | 17.1177 | 60.8 | 114.9 | 393.5 | 0.003 | yes |
| fast | P0 | S05_yaw30/with_cmg | 0 | 3.1448 | 239.3 | 486.7 | 1093.5 | 0.000 | no |
| fast | P0 | S06_pitch30_roll20/no_cmg | 0 | 4.9305 | 44.7 | 93.9 | 397.0 | 0.000 | yes |
| fast | P0 | S06_pitch30_roll20/no_cmg | 1 | 4.9100 | 51.2 | 105.4 | 394.1 | 0.010 | yes |
| fast | P0 | S06_pitch30_roll20/no_cmg | 2 | 4.9335 | 46.2 | 96.3 | 324.3 | 0.007 | yes |
| fast | P0 | S06_pitch30_roll20/with_cmg | 0 | 0.1120 | 272.6 | 582.3 | 1220.4 | 0.007 | no |
| fast | P0 | S06_pitch30_roll20/with_cmg | 1 | 0.1098 | 252.0 | 550.6 | 1229.2 | 0.000 | no |
| fast | P0 | S06_pitch30_roll20/with_cmg | 2 | 0.1038 | 249.2 | 534.6 | 1106.2 | 0.003 | no |
| fast | P0 | S07_sequential/no_cmg | 0 | 2.8022 | 20.4 | 39.2 | 53.0 | 0.000 | yes |
| fast | P0 | S07_sequential/with_cmg | 0 | 0.1916 | 322.5 | 1010.8 | 1216.9 | 0.016 | no |
| fast | P1 | S00_pitch30/no_cmg | 0 | 9.4152 | 50.4 | 133.0 | 409.1 | 0.007 | yes |
| fast | P1 | S00_pitch30/with_cmg | 0 | 0.1542 | 267.4 | 819.6 | 1246.6 | 0.007 | no |
| fast | P1 | S01_pitch_minus30/no_cmg | 0 | 10.2619 | 51.0 | 108.2 | 372.3 | 0.003 | yes |
| fast | P1 | S01_pitch_minus30/with_cmg | 0 | 0.1707 | 272.1 | 647.7 | 1148.8 | 0.010 | no |
| fast | P1 | S06_pitch30_roll20/no_cmg | 0 | 9.0851 | 47.7 | 84.9 | 356.6 | 0.007 | yes |
| fast | P1 | S06_pitch30_roll20/with_cmg | 0 | 0.1517 | 274.2 | 595.0 | 1095.6 | 0.000 | no |
| rti | P0 | S00_pitch30/no_cmg | 0 | 4.7460 | 62.6 | 130.4 | 184.9 | 0.000 | yes |
| rti | P0 | S00_pitch30/no_cmg | 1 | 4.6196 | 55.3 | 99.4 | 121.5 | 0.000 | yes |
| rti | P0 | S00_pitch30/no_cmg | 2 | 4.7477 | 55.7 | 100.8 | 141.3 | 0.000 | yes |
| rti | P0 | S00_pitch30/with_cmg | 0 | 0.1111 | 86.5 | 143.4 | 249.7 | 0.000 | yes |
| rti | P0 | S00_pitch30/with_cmg | 1 | 0.1133 | 80.9 | 119.9 | 128.0 | 0.000 | yes |
| rti | P0 | S00_pitch30/with_cmg | 2 | 0.1117 | 76.4 | 117.4 | 144.9 | 0.000 | yes |
| rti | P0 | S01_pitch_minus30/no_cmg | 0 | 5.1142 | 61.2 | 121.6 | 220.5 | 0.000 | yes |
| rti | P0 | S01_pitch_minus30/no_cmg | 1 | 5.1142 | 56.0 | 100.8 | 122.2 | 0.000 | yes |
| rti | P0 | S01_pitch_minus30/no_cmg | 2 | 5.0009 | 55.9 | 100.0 | 107.1 | 0.000 | yes |
| rti | P0 | S01_pitch_minus30/with_cmg | 0 | 0.1694 | 101.4 | 151.1 | 218.5 | 0.000 | yes |
| rti | P0 | S01_pitch_minus30/with_cmg | 1 | 0.1739 | 95.1 | 121.6 | 156.4 | 0.000 | yes |
| rti | P0 | S01_pitch_minus30/with_cmg | 2 | 0.1520 | 92.0 | 120.8 | 137.5 | 0.000 | yes |
| rti | P0 | S02_pitch10/no_cmg | 0 | 0.5534 | 48.0 | 102.3 | 176.4 | 0.000 | yes |
| rti | P0 | S02_pitch10/with_cmg | 0 | 0.0021 | 88.8 | 140.1 | 183.2 | 0.000 | yes |
| rti | P0 | S03_pitch45/no_cmg | 0 | 10.9549 | 72.8 | 109.7 | 128.0 | 0.000 | yes |
| rti | P0 | S03_pitch45/with_cmg | 0 | 8.4809 | 59.7 | 118.3 | 135.8 | 0.000 | yes |
| rti | P0 | S04_roll30/no_cmg | 0 | 4.0034 | 40.4 | 96.1 | 127.5 | 0.000 | yes |
| rti | P0 | S04_roll30/with_cmg | 0 | 0.0006 | 86.7 | 122.0 | 136.8 | 0.000 | yes |
| rti | P0 | S05_yaw30/no_cmg | 0 | 17.1338 | 60.1 | 101.3 | 120.0 | 0.000 | yes |
| rti | P0 | S05_yaw30/with_cmg | 0 | 5.5871 | 41.1 | 109.5 | 134.8 | 0.000 | yes |
| rti | P0 | S06_pitch30_roll20/no_cmg | 0 | 4.9105 | 50.4 | 101.8 | 117.2 | 0.000 | yes |
| rti | P0 | S06_pitch30_roll20/no_cmg | 1 | 4.9106 | 50.1 | 100.5 | 123.2 | 0.000 | yes |
| rti | P0 | S06_pitch30_roll20/no_cmg | 2 | 4.9106 | 50.2 | 97.7 | 111.0 | 0.000 | yes |
| rti | P0 | S06_pitch30_roll20/with_cmg | 0 | 0.1555 | 83.0 | 120.0 | 132.8 | 0.000 | yes |
| rti | P0 | S06_pitch30_roll20/with_cmg | 1 | 0.1481 | 84.1 | 117.5 | 144.5 | 0.000 | yes |
| rti | P0 | S06_pitch30_roll20/with_cmg | 2 | 0.1582 | 81.1 | 113.2 | 141.5 | 0.000 | yes |
| rti | P0 | S07_sequential/no_cmg | 0 | 2.8025 | 42.6 | 96.3 | 120.5 | 0.000 | yes |
| rti | P0 | S07_sequential/with_cmg | 0 | 0.0641 | 86.1 | 120.2 | 141.0 | 0.000 | yes |
| rti | P1 | S00_pitch30/no_cmg | 0 | 9.2753 | 60.6 | 101.5 | 112.0 | 0.000 | yes |
| rti | P1 | S00_pitch30/with_cmg | 0 | 0.1252 | 86.1 | 119.9 | 141.6 | 0.000 | yes |
| rti | P1 | S01_pitch_minus30/no_cmg | 0 | 10.8090 | 52.1 | 101.8 | 121.0 | 0.000 | yes |
| rti | P1 | S01_pitch_minus30/with_cmg | 0 | 0.1630 | 90.5 | 123.6 | 141.8 | 0.000 | yes |
| rti | P1 | S06_pitch30_roll20/no_cmg | 0 | 8.4396 | 51.8 | 101.9 | 111.1 | 0.000 | yes |
| rti | P1 | S06_pitch30_roll20/with_cmg | 0 | 0.1659 | 84.9 | 122.2 | 140.9 | 0.000 | yes |

## Aggregate over the 8 P0 scenarios (seed 0)

| backend | arm | mean RMSE [deg] | mean step [ms] | p95 step [ms] | max step [ms] | steps within 0.2 s | fallback |
|---|---|---|---|---|---|---|---|
| baseline | no_cmg | 6.2635 | 83.2 | 196.7 | 2669.9 | 4/8 | 0.002 |
| baseline | with_cmg | 0.7334 | 780.9 | 2972.7 | 8387.7 | 0/8 | 0.039 |
| fast | no_cmg | 6.2791 | 41.7 | 94.2 | 397.0 | 8/8 | 0.002 |
| fast | with_cmg | 0.7401 | 251.6 | 648.0 | 1476.8 | 0/8 | 0.014 |
| rti | no_cmg | 6.2773 | 54.8 | 107.4 | 220.5 | 8/8 | 0.000 |
| rti | with_cmg | 1.8214 | 79.1 | 128.1 | 249.7 | 8/8 | 0.000 |
