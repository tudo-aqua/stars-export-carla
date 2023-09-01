@echo off
::Kill the new threads (but no other)
taskkill /F /IM "CarlaUE4-Win64-Shipping.exe"
taskkill /F /IM "CarlaUE4.exe"

::First save current pids with the wanted process name
setlocal EnableExtensions EnableDelayedExpansion

::Get CARLA_HOME variable
call config.bat

::Spawn new Carla instance
echo "Start Carla"
start %CARLA_HOME% -RenderOffScreen
timeout /t 10 /nobreak
set PYTHONPATH=.\..\..\

call .\..\..\venv\Scripts\python.exe ./../../helpers/carla_camera_recorder.py -x 640 -y 480 -s 22 -v 391 -b 57 -e 102
timeout /t 10 /nobreak

::Kill the new threads (but no other)
taskkill /F /IM "CarlaUE4-Win64-Shipping.exe"
taskkill /F /IM "CarlaUE4.exe"
endlocal