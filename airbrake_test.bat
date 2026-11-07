@echo off
rem Guided airbrake test: start a Condor flight, then follow the beeps.
rem Close run_ffb.bat first - both use the same port.
title Condor airbrake field test
cd /d "%~dp0"
python condor_brunner_ffb.py --airbrake-test
pause
