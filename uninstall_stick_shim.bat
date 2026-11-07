@echo off
rem Removes the stick force-feedback shim from Condor's folder (keeps ffb_shim.ini and the log).
title Uninstall Condor stick FFB shim
set "CONDOR=%~1"
if "%CONDOR%"=="" set "CONDOR=C:\Condor3"
if exist "%CONDOR%\dinput8.dll" (
    del "%CONDOR%\dinput8.dll" && echo Removed %CONDOR%\dinput8.dll - Condor uses Windows' DirectInput again after a restart.
) else (
    echo No shim installed in %CONDOR%
)
pause
