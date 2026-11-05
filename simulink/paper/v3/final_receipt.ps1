param([string]$Stamp='20261105_redesign_v1')
$ErrorActionPreference='Continue'
$taskRoot=Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent
Set-Location -LiteralPath $taskRoot
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
$batch=Join-Path $taskRoot "simulink/results/paper_experiments/$Stamp"
& .\simulink\mlab.cmd "addpath('simulink');init_paths;addpath('simulink/paper');addpath('simulink/paper/v3');o=fullfile(repo_root,'simulink/results/paper_experiments/$Stamp');diary(fullfile(o,'config/final_receipt.log'));v3_unit(fullfile(o,'config'));v3_e3_evidence(o);v3_source_manifest(o);a=jsondecode(fileread(fullfile(o,'reports/acceptance.json')));assert(a.paper_acceptance_complete && a.independent_verification.cases_checked==249 && a.independent_verification.reanalysis_pass);disp('V3_FINAL_RECEIPT_PASS');diary off;" *> "$batch/config/final_receipt_launcher.log"
$LASTEXITCODE|Set-Content -LiteralPath "$batch/config/final_receipt_exit.txt"
