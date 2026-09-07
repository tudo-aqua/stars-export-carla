from __future__ import annotations

import subprocess
import sys
import time
from typing import Callable, Any

import carla
import psutil

# Must match the --name given to the container in carla_run.sh.
CARLA_DOCKER_CONTAINER_NAME = "carla_server"


def kill_carla(log: Callable[[str], None] | None = None) -> None:
    """
    Terminate every running Unreal-Engine (CARLA) process on the host.

    On Windows, CARLA runs as a plain native process, so finding and killing
    it by name (below) is enough. On Linux this project instead runs CARLA
    inside a Docker container (see carla_run.sh): the game binary lives in
    the container's own PID namespace under the docker daemon, so killing
    that process directly (even where psutil can see it) doesn't reliably
    stop the container - it can linger holding the host network (--net=host,
    port 2000), blocking the next restart. On Linux we therefore also kill
    the container itself, by name, via the docker CLI.

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
            except psutil.AccessDenied:
                # E.g. a process left behind by a differently-invoked docker
                # run (different UID mapping) that this user can't signal
                # directly. Not fatal here - on Linux the docker-kill below
                # (or a manual cleanup) is the real way to remove it; don't
                # let it abort the whole restart/transform.
                _log(f">> [CARLA] Warning: no permission to kill {proc.info['name']} "
                     f"(pid={proc.pid}); leaving it running")

    if sys.platform.startswith("linux"):
        try:
            result = subprocess.run(
                ["docker", "kill", CARLA_DOCKER_CONTAINER_NAME],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=10,
            )
            if result.returncode == 0:
                _log(f">> [CARLA] Killed docker container '{CARLA_DOCKER_CONTAINER_NAME}'")
            # A non-zero exit here just means no such container was running,
            # which is the common/expected case - nothing to log.
        except FileNotFoundError:
            pass  # docker CLI not installed/on PATH; nothing we can do
        except subprocess.TimeoutExpired:
            _log(">> [CARLA] 'docker kill' timed out")


def start_carla(
        exe: str,
        *,
        render_off_screen: bool = True,
        quality_low: bool = True,
        map_name: str | None = None,
        boot: float = 20.0,
        log: Callable[[str], None] | None = None,
) -> None:
    """
    Launch a CARLA server in its own process group and optionally set the map
    using PythonAPI/util/config.py after an optional boot wait.
    """
    _log = log or print

    cmd = [exe]
    if render_off_screen:
        cmd.append("-RenderOffScreen")
    if quality_low:
        cmd.append("-quality-level=Low")

    kwargs: dict[str, Any]
    if sys.platform.startswith("win"):
        kwargs = dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        kwargs = {}

    subprocess.Popen(cmd, **kwargs)

    # If a map is requested, wait for UE to boot, then switch maps via config.py
    if boot > 0:
        _log(f">> [CARLA] Waiting {boot:.1f}s for CARLA to boot")
        time.sleep(boot)
    if map_name:
        ok = set_map(map_name, log=_log)
        if not ok:
            _log(f">> [CARLA] Warning: failed to set map '{map_name}' via config.py")


def restart_carla(
        exe: str,
        *,
        render_off_screen: bool = True,
        render_quality_low: bool = True,
        map_name: str | None = None,
        cooldown: float = 5.0,
        boot: float = 20.0,
        log: Callable[[str], None] | None = None,
) -> None:
    """
    Hard-restart the CARLA server and wait until it is ready; optionally load a map.

    The routine is a convenience wrapper around:
    kill_carla -> cool-down -> start_carla (with flags/map) -> boot-wait (handled in start_carla).

    Parameters
    ----------
    exe : str
        Fully qualified path to the CARLA executable.
    render_off_screen : bool, default False
        Forwarded to ``start_carla``.
    render_quality_low : bool, default False
        Forwarded to ``start_carla``.
    map_name : str | None, default None
        Forwarded to ``start_carla`` to load after boot.
    cooldown : float, default 5.0
        Seconds to wait after killing existing instances but before spawning a new one.
    boot : float, default 20.0
        Seconds to wait after launching CARLA before trying to load the map.
        Only relevant when ``map_name`` is provided.
    log : Callable[[str], None] | None, default None
        Optional logger (defaults to ``print``).
    """
    _log = log or print

    _log(">> [CARLA] Killing existing instances")
    kill_carla(_log)

    if cooldown > 0:
        _log(f">> [CARLA] Waiting {cooldown:.1f}s before restart")
        time.sleep(cooldown)

    _log(">> [CARLA] Starting new server")
    start_carla(
        exe,
        render_off_screen=render_off_screen,
        quality_low=render_quality_low,
        map_name=map_name,
        boot=boot,
        log=_log,
    )

    _log(">> [CARLA] Server should now be ready")


def restart_and_connect(
        exe: str,
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 600,
        *,
        render_off_screen: bool = True,
        render_quality_low: bool = True,
        map_name: str | None = None,
        cooldown: float = 5,
        boot: float = 20,
        log: Callable[[str], None] | None = None
):
    """
    Restart CARLA and return an active `carla.Client`.

    Parameters
    ----------
    exe : str
        Location of the CARLA executable passed through to `restart_carla`.
    host : str, default ``"localhost"``
        Connection endpoint for the CARLA RPC server.
    port: int, default ``2000``
        Connection endpoint port for the CARLA RPC server.
    timeout : float, default 60
        Seconds before a socket operation on the client aborts.
    render_off_screen, render_quality_low : bool
        Forwarded to `restart_carla`.
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
    restart_carla(
        exe,
        render_off_screen=render_off_screen,
        render_quality_low=render_quality_low,
        map_name=map_name,
        cooldown=cooldown,
        boot=boot,
        log=_log,
    )

    _log(">> [CARLA] Connecting to server")
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    _log(">> [CARLA] Connected.")
    return client


def set_map(
        map_name: str,
        *,
        host: str = "localhost",
        port: int = 2000,
        log: Callable[[str], None] | None = None,
) -> bool:
    """
    Invoke CARLA's config.py to change the running map, e.g.:
        python PythonAPI/util/config.py --host <host> --port <port> --map Town05
    Returns True on success, False otherwise.
    """
    _log = log or print
    _log(f">> [CARLA] Changing map to '{map_name}'")
    client = carla.Client(host=host, port=port)
    client.set_timeout(20.0)
    client.load_world_if_different(map_name)
    try:
        client.get_available_maps()
        _log(f">> [CARLA] Loaded map '{map_name}'")
        return True
    except RuntimeError:
        _log(f">> [CARLA] Failed loading map '{map_name}'")
        return False
