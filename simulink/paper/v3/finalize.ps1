param([string]$Stamp='20261105_redesign_v1',[int[]]$WaitMatlabPid=@())
$ErrorActionPreference='Stop'
$taskRoot=Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent
Set-Location -LiteralPath $taskRoot
$batch=Join-Path $taskRoot "simulink/results/paper_experiments/$Stamp"
foreach($ownedPid in $WaitMatlabPid) { Wait-Process -Id $ownedPid -ErrorAction SilentlyContinue }
Start-Sleep -Seconds 3
@{started=(Get-Date).ToString('o');waited_matlab_pids=$WaitMatlabPid;mode='Fresh MATLAB independent numerical reanalysis, timing/physical audit, plots and acceptance'}|ConvertTo-Json|Set-Content -LiteralPath "$batch/config/finalization_started.json" -Encoding utf8
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
$ErrorActionPreference='Continue' # Native MATLAB startup stderr must not terminate the PowerShell owner.
& .\simulink\mlab.cmd "addpath('simulink');init_paths;addpath('simulink/paper');addpath('simulink/paper/v3');o=fullfile(repo_root,'simulink/results/paper_experiments/$Stamp');diary(fullfile(o,'config/final_acceptance.log'));v3_compact_derived(o);v3_acceptance(o);a=jsondecode(fileread(fullfile(o,'reports/acceptance.json')));assert(a.paper_acceptance_complete,'v3:acceptance','Independent acceptance incomplete; inspect evidence and repair');disp('V3_ALL_ACCEPTANCE_PASS');diary off;" *> "$batch/config/final_acceptance_launcher.log"
$LASTEXITCODE|Set-Content -LiteralPath "$batch/config/exit_final_acceptance.txt"
