from __future__ import annotations
import os, sys, subprocess, time
import psutil
import carla
from typing import Callable, Any


def kill_carla(log: Callable[[str], None] | None = None) -> None:
    """
    Terminate every running Unreal-Engine (CARLA) process on the host.

    Parameters
    ----------
    log : Callable[[str], None], optional
        Logging callback that will receive status messages
        (defaults to `print`).  The callback is invoked **before**
        each process is killed, so you always know which binary
        was terminated (e.g. ``CarlaUE4-Win64-Shipping.exe``).
    """
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
    """
    Launch a head-less CARLA server in its **own** process group.

    Parameters
    ----------
    exe : str
        Path to the ``CarlaUE4`` executable or launcher script.
    """
    cmd = [exe, "-RenderOffScreen"]
    kwargs: dict[str, Any]
    if sys.platform.startswith("win"):
        kwargs = dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        kwargs = dict(preexec_fn=os.setsid)
    subprocess.Popen(cmd, **kwargs)


def restart_carla(
        exe: str,
        *,
        cooldown: float = 5,
        boot: float = 20,
        log: Callable[[str], None] | None = None
) -> None:
    """
    Hard-restart the CARLA server and wait until it is ready.

    The routine is a convenience wrapper around `kill_carla` ->
    *cool-down* -> `start_carla` -> *boot-wait*.

    Parameters
    ----------
    exe : str
        Fully qualified path to the CARLA executable.
    cooldown : float, default 5
        Seconds to wait *after* killing existing instances but *before*
        spawning a new one.  Gives the OS time to release sockets/handles.
    boot : float, default 20
        Seconds to wait *after* launching CARLA so UE4 can finish loading maps.
    log : Callable[[str], None], optional
        Logging callback (defaults to `print`).
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
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 60,
        cooldown: float = 5,
        boot: float = 20,
        log: Callable[[str], None] | None = None
):
    """
    Restart CARLA and return an active `carla.Client`.

    Parameters
    ----------
    exe : str
        Location of the CARLA executable passed through to
        `restart_carla`.
    host : str default ``"localhost"``
        Connection endpoint for the CARLA RPC server.
    port: int, default ``2000``
        Connection endpoint port for the CARLA RPC server.
    timeout : float, default 60
        Seconds before a socket operation on the client aborts.
    cooldown, boot : float
        Forwarded verbatim to `restart_carla`.
    log : Callable[[str], None], optional
        Logger callback (defaults to `print`).

    Returns
    -------
    carla.Client
        A connected client of Carla ready for world interaction.
    """
    _log = log or print
    restart_carla(exe, cooldown=cooldown, boot=boot, log=_log)

    _log(">> [CARLA] Connecting to server …")
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    _log(">> [CARLA] Connected.")
    return client
