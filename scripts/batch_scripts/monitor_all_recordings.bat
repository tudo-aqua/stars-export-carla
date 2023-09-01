@echo off
echo monitor_all_recordings.bat
setlocal enabledelayedexpansion
set ABS_PATH=%CD%
echo abs: %ABS_PATH%
for /D %%s in ("./../../generated-data/recordings"\*) do (
    echo "Map folder: %%s"
    for %%f in (%%s\*) do (
        call :monitor %%s %%f
    )
)
endlocal


:monitor
set MAP_FOLDER=%1
set FILE_PATH=%2
Echo.%FILE_PATH% | findstr /C:"weather">nul && (
    echo Skip weather data at %FILE_PATH%
) || (
    echo Current file %FILE_PATH%
    echo monitor_recording.bat %MAP_FOLDER% %FILE_PATH%
    call monitor_recording.bat %MAP_FOLDER% %FILE_PATH%
    echo "====================================================="
)
:End