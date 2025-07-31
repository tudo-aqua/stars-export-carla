#!/usr/bin/env python3
import os, sys, subprocess
import psutil

def kill_carla() -> None:
    """Kill every running CARLA UE4 process (cross-platform)."""
    for proc in psutil.process_iter(["name"]):
        if proc.info["name"] in (
                "CarlaUE4-Win64-Shipping.exe", "CarlaUE4.exe",
                "CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
            try:
                proc.kill()
            except psutil.NoSuchProcess:
                pass


def start_carla(exe: str) -> None:
    """Launch CARLA headless in its own process-group."""
    cmd = [exe, "-RenderOffScreen"]
    kw = dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP) \
        if sys.platform.startswith("win") else dict(preexec_fn=os.setsid)
    subprocess.Popen(cmd, **kw)
