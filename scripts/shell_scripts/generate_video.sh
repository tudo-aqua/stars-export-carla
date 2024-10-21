#!/bin/bash

# Kill the new threads (but no other)
pkill -f "CarlaUE4-Linux-Shipping"
pkill -f "CarlaUE4"

# Get CARLA_HOME variable by sourcing config.sh (assuming you have a shell script equivalent)
source ./config.sh

# Spawn new Carla instance
echo "Start Carla"
"$CARLA_HOME" -RenderOffScreen &
sleep 10

# Set PYTHONPATH
export PYTHONPATH=../../../

# Run the Python script
../../venv/bin/python3 ../../helpers/carla_camera_recorder.py -x 1920 -y 1080 -e 1 -p "$1" -d ../../

sleep 10

# Kill the new threads (but no other)
pkill -f "CarlaUE4-Linux-Shipping"
pkill -f "CarlaUE4"
