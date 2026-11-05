param([int]$Amplitude,[string]$Stamp='20261105_redesign_v1')
$taskRoot=Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent
Set-Location -LiteralPath $taskRoot
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
$batch=Join-Path $taskRoot "simulink/results/paper_experiments/$Stamp"
& .\simulink\mlab.cmd "addpath('simulink');init_paths;addpath('simulink/paper');addpath('simulink/paper/v3');diary('$($batch.Replace('\','/'))/config/console_E4_A$Amplitude.log');v3_e4_partition('$Stamp',$Amplitude);diary off;" *> "$batch/config/launcher_E4_A$Amplitude.log"
$LASTEXITCODE|Set-Content -LiteralPath "$batch/config/exit_E4_A$Amplitude.txt"
