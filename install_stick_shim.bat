@echo off
rem Installs the stick force-feedback shim into Condor's folder.
rem Takes effect the next time Condor starts. Undo with uninstall_stick_shim.bat.
title Install Condor stick FFB shim
cd /d "%~dp0"
set "CONDOR=%~1"
if "%CONDOR%"=="" set "CONDOR=C:\Condor3"
if not exist "%CONDOR%\Condor.exe" (
    echo Condor.exe not found in %CONDOR%
    echo Usage: install_stick_shim.bat "D:\path\to\Condor3"
    pause
    exit /b 1
)
copy /y "stick_shim\dinput8.dll" "%CONDOR%\dinput8.dll" >nul || goto :fail
if exist "%CONDOR%\ffb_shim.ini" (
    echo Kept your existing %CONDOR%\ffb_shim.ini
) else (
    copy /y "stick_shim\ffb_shim.ini" "%CONDOR%\ffb_shim.ini" >nul || goto :fail
)
echo Installed into %CONDOR%
echo  - restart Condor to load it
echo  - strengths: %CONDOR%\ffb_shim.ini  (applies live)
echo  - what Condor sends the stick: %CONDOR%\ffb_shim.log
pause
exit /b 0
:fail
echo Copy failed - is the folder writable?
pause
exit /b 1
