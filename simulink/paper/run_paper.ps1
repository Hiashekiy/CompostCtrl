param([string]$Mode='all',[string]$Stamp='20261105_paper_v2')
$ErrorActionPreference='Stop'
Set-Location -LiteralPath (Split-Path $PSScriptRoot -Parent | Split-Path -Parent)
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
$logDir=Join-Path (Get-Location) "simulink/results/paper_experiments/$Stamp"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
& .\simulink\mlab.cmd "addpath('simulink'); init_paths; addpath('simulink/paper'); diary('$($logDir.Replace('\','/'))/console.log'); run_paper_suite('$Mode','$Stamp'); diary off;" *> "$logDir/launcher.log"
$LASTEXITCODE | Set-Content "$logDir/exit_code.txt"

