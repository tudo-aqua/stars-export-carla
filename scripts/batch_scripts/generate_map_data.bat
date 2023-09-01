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

call .\..\..\venv\Scripts\python.exe ./../../helpers/carla_map_generator.py -u true
timeout /t 10 /nobreak

::Kill the new threads (but no other)
taskkill /F /IM "CarlaUE4-Win64-Shipping.exe"
taskkill /F /IM "CarlaUE4.exe"
endlocal