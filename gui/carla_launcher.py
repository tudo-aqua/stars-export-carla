#!/usr/bin/env python3
"""
CARLA process helpers

• kill_carla()           – terminate every UE4 process
• start_carla(exe)       – launch head-less CARLA
• restart_carla(exe, …)  – kill → wait → start → wait  (with logging)
• restart_and_connect()  – above + return carla.Client
"""
from __future__ import annotations
import os, sys, subprocess, time
import psutil
from typing import Callable, Any


# ────────────────────────────────────────────────────────────────────
# basic helpers
# ────────────────────────────────────────────────────────────────────
def kill_carla(log: Callable[[str], None] | None = None) -> None:
    """Kill every running CARLA UE4 process (cross-platform)."""
    _log = log or print
    for proc in psutil.process_iter(["name"]):
        if proc.info["name"] in (
                "CarlaUE4-Win64-Shipping.exe", "CarlaUE4.exe",
                "CarlaUE4-Linux-Shipping", "CarlaUE4.sh"
        ):
            try:
                _log(f">> [CARLA] Killing existing instance: {proc.info['name']}")
                proc.kill()
            except psutil.NoSuchProcess:
                pass


def start_carla(exe: str) -> None:
    """Launch CARLA head-less in its own process group."""
    cmd = [exe, "-RenderOffScreen"]
    kwargs: dict[str, Any]
    if sys.platform.startswith("win"):
        kwargs = dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        kwargs = dict(preexec_fn=os.setsid)
    subprocess.Popen(cmd, **kwargs)


# ────────────────────────────────────────────────────────────────────
# reusable routines for workers, now with logging
# ────────────────────────────────────────────────────────────────────
def restart_carla(
        exe: str,
        *,
        cooldown: float = 5,
        boot: float = 20,
        log: Callable[[str], None] | None = None
) -> None:
    """
    Kill any existing CARLA, wait *cooldown* seconds,
    start a fresh one, wait *boot* seconds until it’s ready.

    Parameters
    ----------
    exe : str
        Path to CarlaUE4 executable.
    cooldown : float
        Seconds to wait after killing before re-starting.
    boot : float
        Seconds to wait after launch so the server is fully ready.
    log : callable, optional
        Function that consumes a text message (default: built-in `print`).
    """
    _log = log or print

    _log(">> [CARLA] Killing existing instances …")
    kill_carla(_log)

    if cooldown > 0:
        _log(f">> [CARLA] Waiting {cooldown:.1f}s before restart")
        time.sleep(cooldown)

    _log(">> [CARLA] Starting new server …")
    start_carla(exe)

    if boot > 0:
        _log(f">> [CARLA] Waiting {boot:.1f}s for CARLA to boot")
        time.sleep(boot)

    _log(">> [CARLA] Server should now be ready")


def restart_and_connect(
        exe: str,
        *,
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 60,
        cooldown: float = 5,
        boot: float = 20,
        log: Callable[[str], None] | None = None
):
    """
    Convenience wrapper: restart CARLA (see above), then create and
    return a connected `carla.Client`.
    """
    _log = log or print
    restart_carla(exe, cooldown=cooldown, boot=boot, log=_log)

    import carla  # late import – only if we actually need to connect
    _log(">> [CARLA] Connecting to server …")
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    _log(">> [CARLA] Connected.")
    return client
