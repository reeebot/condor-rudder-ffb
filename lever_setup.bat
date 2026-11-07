@echo off
rem Finds your airbrake lever and saves it to settings.ini. Run once.
title Airbrake lever setup
cd /d "%~dp0"
python condor_brunner_ffb.py --lever-setup
pause
