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
rem ---- C toolchain -----------------------------------------------------------
rem  The MathWorks MinGW add-on is not installed, but a standalone MinGW-w64
rem  (w64devkit, GCC 15.2) is; MATLAB's mingw64.xml locates it through
rem  MW_MINGW64_LOC and `mex` works (it only warns that the version is not on the
rem  officially supported list).  Set it here so that every headless run can
rem  compile S-functions / MEX files without extra environment juggling.
if not defined MW_MINGW64_LOC if exist "D:\w64devkit\bin\gcc.exe" set "MW_MINGW64_LOC=D:\w64devkit"
cd /d "%REPO%"
"%MATLAB_HOME%\bin\matlab.exe" -nosplash -noFigureWindows -batch %*
set "MATLAB_EXIT=%ERRORLEVEL%"
endlocal & exit /b %MATLAB_EXIT%
