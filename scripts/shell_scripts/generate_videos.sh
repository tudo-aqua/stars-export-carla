#!/bin/bash

# Define the base folder path
BASE_DIR=../../../scenarios

# Iterate over all folders in the base directory
for folder in "$BASE_DIR"/*/; do
    # Find the .log file in each folder and call generate_video.bat (assumed to be a bash script or an executable)
    for log_file in "$folder"*.log; do
        if [ -f "$log_file" ]; then
            echo "Processing log file: $log_file"
            ./generate_video.bat "$log_file"
        fi
    done
done
