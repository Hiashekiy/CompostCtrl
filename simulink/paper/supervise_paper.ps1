param([string]$Stamp='20261105_paper_v2',[int]$WaitPid=0)
$ErrorActionPreference='Continue'
Set-Location -LiteralPath (Split-Path $PSScriptRoot -Parent | Split-Path -Parent)
if ($WaitPid -gt 0) { Wait-Process -Id $WaitPid -ErrorAction SilentlyContinue }
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
$dir=Join-Path (Get-Location) "simulink/results/paper_experiments/$Stamp"
for ($attempt=1; $attempt -le 3; $attempt++) {
 & .\simulink\mlab.cmd "addpath('simulink'); init_paths; addpath('simulink/paper'); diary('simulink/paper_supervisor_$attempt.log'); run_paper_suite('all','$Stamp'); paper_finish('$Stamp'); diary off;" *> "$dir/supervisor_$attempt.log"
 $code=$LASTEXITCODE
 @{attempt=$attempt;exit_code=$code;updated=(Get-Date).ToString('o')} | ConvertTo-Json | Set-Content "$dir/supervisor_status.json"
 if ($code -eq 0) { break }
}

