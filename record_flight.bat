@echo off
rem Records all Condor UDP data to condor_record.csv. Press Ctrl+C when done.
rem Close run_ffb.bat first - both use the same port.
title Condor flight recorder
cd /d "%~dp0"
python condor_brunner_ffb.py --record
pause
