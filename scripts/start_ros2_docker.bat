@echo off
REM Build & enter ROS2 Humble container for V0.1
cd /d "%~dp0\.."

echo [1/3] Export office smap to Nav2 map...
set PYTHONPATH=%CD%\ros2_ws\src\agv_bridge;%PYTHONPATH%
python scripts\export_smap_to_navmap.py --out maps\office_nav

echo [2/3] Build Docker image agv_v01_humble (first time may take long)...
docker compose -f docker\docker-compose.yml build ros2_humble
if errorlevel 1 (
  echo Docker build failed. Is Docker Desktop running?
  exit /b 1
)

echo [3/3] Start container shell...
echo   Host Mock Web should be running: scripts\start_web_sim.bat
echo   Inside container:
echo     colcon build --symlink-install --packages-select agv_control
echo     source install/setup.bash
echo     ros2 run agv_control agv_control_bridge --ros-args -p backend:=tcp -p agv_host:=host.docker.internal
docker compose -f docker\docker-compose.yml run --rm ros2_humble bash
