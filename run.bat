@echo off
rem Condor -> Brunner rudder force feedback. Keep this window open while you fly.
rem Start CLS2Sim (connected to the pedals) first. Ctrl+C here to stop cleanly.
rem Tuning: edit settings.ini in this folder - saved changes apply live.
title Condor rudder FFB
cd /d "%~dp0"
python condor_brunner_ffb.py
pause
