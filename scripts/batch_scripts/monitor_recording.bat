@echo off
::Kill the new threads (but no other)
taskkill /F /IM "CarlaUE4-Win64-Shipping.exe"
taskkill /F /IM "CarlaUE4.exe"

timeout /t 2 /nobreak

::First save current pids with the wanted process name
setlocal EnableExtensions EnableDelayedExpansion

::Get CARLA_HOME variable
call config.bat

::Spawn new Carla instance
echo "Start Carla"
start %CARLA_HOME% -RenderOffScreen
echo Wait 20 seconds for Carla to start properly
timeout /t 20 /nobreak

set PYTHONPATH=.\..\..\

echo Got Folder: %1
echo Got File: %2

echo "Call Python code"
call .\..\..\venv\Scripts\python.exe ./../../helpers/carla_monitor.py --file=%2
timeout /t 10 /nobreak

::Kill the new threads (but no other)
taskkill /F /IM "CarlaUE4-Win64-Shipping.exe"
taskkill /F /IM "CarlaUE4.exe"
endlocal