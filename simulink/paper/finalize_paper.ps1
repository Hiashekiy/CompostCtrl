param([int]$WaitPid=58336)
Set-Location -LiteralPath (Split-Path $PSScriptRoot -Parent | Split-Path -Parent)
Wait-Process -Id $WaitPid -ErrorAction SilentlyContinue
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
& .\simulink\mlab.cmd "addpath('simulink'); init_paths; addpath('simulink/paper'); diary('simulink/paper_final_validation.log'); test_paper_core; if ~isfile('simulink/results/paper_experiments/20261105_paper_v2/batch_completed.json'),paper_finish('20261105_paper_v2');else,run_paper_suite('report','20261105_paper_v2');end; diary off;" *> 'simulink/results/paper_experiments/20261105_paper_v2/finalization.log'
$LASTEXITCODE | Set-Content 'simulink/results/paper_experiments/20261105_paper_v2/finalization_exit_code.txt'

