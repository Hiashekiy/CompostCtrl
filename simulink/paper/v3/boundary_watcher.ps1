param([int]$MainPid,[string]$StartUtc)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent
$batch=Join-Path $taskRoot 'simulink/results/paper_experiments/20261105_redesign_v1'
while ($true) {
 $states=@(Get-ChildItem -LiteralPath (Join-Path $batch 'E1/cases') -Recurse -Filter run_status.json -ErrorAction SilentlyContinue)
 if ($states.Count -eq 144) {
  $running=@($states|Where-Object {(Get-Content -LiteralPath $_.FullName -Raw|ConvertFrom-Json).status -eq 'RUNNING'})
  $raw=@(Get-ChildItem -LiteralPath (Join-Path $batch 'E1/cases') -Recurse -Filter sim_output.mat)
  if ($running.Count -eq 0 -and $raw.Count -eq 144) {
   $owned=Get-Process -Id $MainPid -ErrorAction SilentlyContinue
   if ($owned -and $owned.ProcessName -eq 'MATLAB' -and $owned.StartTime.ToUniversalTime().ToString('o') -eq $StartUtc) {
    @{reason='Assigned E2/E3/E1 matrices complete; E4 independently owned by E4 worker. Stop main only during derived report stage to prevent duplicate E4 execution.';E1_statuses=144;E1_raw=144;main_pid=$MainPid;time=(Get-Date).ToString('o');controlled_matrix_boundary_stop=$true}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $batch 'config/controlled_boundary_stop.json') -Encoding utf8
    Stop-Process -Id $MainPid
    Wait-Process -Id $MainPid -Timeout 15 -ErrorAction SilentlyContinue
    foreach ($lock in @(Get-ChildItem -LiteralPath $batch -Recurse -Force -Filter '*.lock')) {
     try { $owner=Get-Content -LiteralPath $lock.FullName -Raw|ConvertFrom-Json; if ($owner.pid -eq $MainPid) { Remove-Item -LiteralPath $lock.FullName -Force } } catch { }
    }
   }
   break
  }
 }
 if (-not (Get-Process -Id $MainPid -ErrorAction SilentlyContinue)) { break }
 Start-Sleep -Seconds 1
}
