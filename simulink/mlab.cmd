@echo off
rem ===========================================================================
rem  mlab.cmd -- run MATLAB R2024a headless with all per-user writable state
rem  redirected into this workspace.
rem
rem  Reason: the DSH file sandbox only allows writes under the session workspace,
rem  and MATLAB refuses to start when it cannot write %APPDATA%\MathWorks
rem  ("failed to load settings errors_warnings plugin").  APPDATA / LOCALAPPDATA /
rem  TMP / TEMP / MATLAB_PREFDIR are therefore pointed at .matlab_* inside the repo.
rem
rem  Usage:  simulink\mlab.cmd "cd('simulink'); check_env"
rem          simulink\mlab.cmd -sd . -batch mode_script
rem ===========================================================================
setlocal
set "REPO=%~dp0.."
for %%I in ("%REPO%") do set "REPO=%%~fI"
set "APPDATA=%REPO%\.matlab_appdata"
set "LOCALAPPDATA=%REPO%\.matlab_appdata"
set "MATLAB_PREFDIR=%REPO%\.matlab_prefs"
if not exist "%REPO%\.matlab_temp" mkdir "%REPO%\.matlab_temp"
set "TMP=%REPO%\.matlab_temp"
set "TEMP=%REPO%\.matlab_temp"
set "MATLAB_LOGGING_FOLDER=%REPO%\.matlab_temp\log"
cd /d "%REPO%"
"%MATLAB_HOME%\bin\matlab.exe" -nosplash -noFigureWindows -batch %*
endlocal
