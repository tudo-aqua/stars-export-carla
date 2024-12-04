@echo off
echo monitor_all_recordings.bat
setlocal enabledelayedexpansion
set ABS_PATH=%CD%
echo abs: %ABS_PATH%
for /D %%s in ("./../../scenarios"\*) do (
    echo "Map folder: %%s"
        echo monitor_recording.bat %%s
        call monitor_recording.bat %%s
)
endlocal