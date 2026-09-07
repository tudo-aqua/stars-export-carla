#!/bin/bash
# Make sure no stale container (e.g. left over from an unclean previous
# kill) is still holding the name or the host network before starting a
# fresh one.
docker rm -f carla_server >/dev/null 2>&1

docker run \
    --name=carla_server \
    --rm \
    --runtime=nvidia \
    --net=host \
    --user=$(id -u):$(id -g) \
    --env=DISPLAY=$DISPLAY \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume ~/workspace/carlarecordings/:/workspace/recordings \
    carlasim/carla:0.9.16 bash CarlaUE4.sh -nosound "$@"
