param([string]$Mode='all',[string]$Stamp='20261105_redesign_v1')
$ErrorActionPreference='Continue'
$taskRoot=Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent
Set-Location -LiteralPath $taskRoot
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
$batch=Join-Path $taskRoot "simulink/results/paper_experiments/$Stamp"
New-Item -ItemType Directory -Force -Path "$batch/config" | Out-Null
& .\simulink\mlab.cmd "addpath('simulink'); init_paths; addpath('simulink/paper'); addpath('simulink/paper/v3'); diary('$($batch.Replace('\','/'))/config/console_$Mode.log'); paper_v3('$Mode','$Stamp'); diary off;" *> "$batch/config/launcher_$Mode.log"
$LASTEXITCODE | Set-Content "$batch/config/exit_$Mode.txt"
