@echo off
REM Windows: true-scene sim web (smap + Three.js, no ROS2)
cd /d "%~dp0\.."
set PYTHONPATH=%CD%\ros2_ws\src\agv_bridge;%PYTHONPATH%
python scripts\run_web_sim.py
