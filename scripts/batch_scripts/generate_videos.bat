@echo off
setlocal EnableDelayedExpansion

:: Define the base folder path
set "BASE_DIR=.\..\..\scenarios"

:: Iterate over all folders in the base directory
for /d %%F in ("%BASE_DIR%\*") do (
    :: Find the .log file and call generate_video.bat with it
    for %%L in ("%%F\*.log") do (
        echo Processing log file: %%L
        call generate_video.bat "%%L"
    )
)

endlocal
