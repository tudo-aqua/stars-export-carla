from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable

import psutil

from carla_interaction_gui.carla_versions import is_start_script, python_api_env, start_script_for
from carla_interaction_gui.gui.constants import (
    CARLA_BOOT_TIMEOUT_S,
    CARLA_DOCKER_CONTAINER_NAME,
    CARLA_DOCKER_RECORDINGS_CONTAINER_DIR,
    CARLA_DOCKER_RECORDINGS_HOST_DIR,
    CARLA_DOCKER_RUN_ARGS,
    CARLA_DOCKER_SERVER_ARGS,
    CARLA_STOP_TIMEOUT_S,
)

# The CARLA Python API (`import carla`) is only imported inside the functions
# that talk to the server: those run in worker subprocesses that get the API
# version matching the server put on PYTHONPATH (see carla_versions). The GUI
# process imports this module too, for kill_carla/restart_carla, and must not
# load any CARLA API itself.

# Where every task expects the server (the CARLA defaults).
CARLA_HOST = "localhost"
CARLA_PORT = 2000

_CARLA_PROCESS_NAMES = (
    "CarlaUE4-Linux-Shipping", "CarlaUE4.sh",
    "CarlaUnreal-Linux-Shipping", "CarlaUnreal.sh",
)

# Run in a subprocess with the selected Python API: succeeds once the server
# serves requests. The RPC port opens a few seconds before that, so an open
# port alone isn't enough.
_READY_PROBE = (
    "import sys, carla\n"
    "client = carla.Client(sys.argv[1], int(sys.argv[2]))\n"
    "client.set_timeout(2.0)\n"
    "client.get_world()\n"
)


def kill_carla(log: Callable[[str], None] | None = None) -> None:
    """
    Terminate every running CARLA server on the host.

    CARLA launched from a start script runs as a plain host process, so
    finding and killing it by name (below) is enough. CARLA launched from a
    docker image instead lives in the container's own PID namespace under the
    docker daemon, so killing that process directly (even where psutil can see
    it) doesn't reliably stop the container - it can linger holding the host
    network (--net=host, port 2000), blocking the next restart. The container
    itself is therefore also killed, by name, via the docker CLI.

    Parameters
    ----------
    log : Callable[[str], None], optional
        Logging callback that will receive status messages
        (defaults to `print`).  The callback is invoked **before**
        each process is killed, so you always know which binary
        was terminated (e.g. ``CarlaUE4-Linux-Shipping``).
    """
    _log = log or print
    for proc in psutil.process_iter(["name"]):
        if proc.info["name"] in _CARLA_PROCESS_NAMES:
            try:
                _log(f">> [CARLA] Killing existing instance: {proc.info['name']}")
                proc.kill()
            except psutil.NoSuchProcess:
                pass
            except psutil.AccessDenied:
                # E.g. a process inside a docker container running under a
                # different UID mapping that this user can't signal directly.
                # Not fatal here - the docker-kill below (or a manual cleanup)
                # is the real way to remove it; don't let it abort the whole
                # restart/transform.
                _log(f">> [CARLA] Warning: no permission to kill {proc.info['name']} "
                     f"(pid={proc.pid}); leaving it running")

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


def _port_open(host: str = CARLA_HOST, port: int = CARLA_PORT) -> bool:
    try:
        socket.create_connection((host, port), timeout=0.5).close()
        return True
    except OSError:
        return False


def _carla_still_running() -> str | None:
    """What of a previous CARLA server is still around, if anything."""
    try:
        if subprocess.run(["docker", "ps", "-q", "-f", f"name=^{CARLA_DOCKER_CONTAINER_NAME}$"],
                          capture_output=True, text=True, timeout=10).stdout.strip():
            return f"docker container '{CARLA_DOCKER_CONTAINER_NAME}'"
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    for proc in psutil.process_iter(["name"]):
        if proc.info["name"] in _CARLA_PROCESS_NAMES:
            return f"process {proc.info['name']} (pid={proc.pid})"
    if _port_open():
        return f"something listening on port {CARLA_PORT}"
    return None


def wait_until_stopped(timeout: float = CARLA_STOP_TIMEOUT_S, log: Callable[[str], None] | None = None) -> bool:
    """
    Wait until no CARLA server is left (container, process or port 2000), so
    a new one can bind the port. Returns False if one is still around after
    `timeout` seconds.
    """
    _log = log or print
    deadline = time.monotonic() + timeout
    while (leftover := _carla_still_running()) is not None:
        if time.monotonic() > deadline:
            _log(f">> [CARLA] Warning: {leftover} still running after {timeout:.0f}s")
            return False
        time.sleep(0.2)
    return True


def wait_until_ready(
        server: subprocess.Popen,
        *,
        timeout: float = CARLA_BOOT_TIMEOUT_S,
        python_api: str | None = None,
        log: Callable[[str], None] | None = None,
) -> None:
    """
    Wait until the just-started CARLA server serves requests. Raises if the
    server process exits while booting or isn't ready after `timeout` seconds
    (a first start of an image can take much longer than later ones, e.g.
    while compiling shaders).

    The check imports the CARLA Python API in a subprocess: with `python_api`
    given that version (for callers in the GUI process, which must not import
    it), otherwise whatever this process' environment provides.
    """
    _log = log or print
    env = python_api_env(python_api, _log) if python_api else None
    started = time.monotonic()
    next_report = started + 10
    while True:
        exit_code = server.poll()
        if exit_code is not None:
            raise RuntimeError(f"CARLA server exited with code {exit_code} while booting")
        if _port_open():
            try:
                probe = subprocess.run([sys.executable, "-c", _READY_PROBE, CARLA_HOST, str(CARLA_PORT)],
                                       env=env, capture_output=True, timeout=15)
                if probe.returncode == 0:
                    _log(f">> [CARLA] Server ready after {time.monotonic() - started:.1f}s")
                    return
            except subprocess.TimeoutExpired:
                pass
        now = time.monotonic()
        if now - started > timeout:
            raise TimeoutError(f"CARLA server not ready after {timeout:.0f}s")
        if now >= next_report:
            _log(f">> [CARLA] Still waiting for the server to boot ({now - started:.0f}s)")
            next_report = now + 10
        time.sleep(0.5)


def _docker_run_command(image: str, server_args: list[str]) -> list[str]:
    """The `docker run` command starting a CARLA server from the given image."""
    recordings_host_dir = Path(CARLA_DOCKER_RECORDINGS_HOST_DIR).expanduser()
    # docker would otherwise create a missing mount source itself, owned by root.
    recordings_host_dir.mkdir(parents=True, exist_ok=True)
    return [
        "docker", "run",
        f"--name={CARLA_DOCKER_CONTAINER_NAME}",
        *CARLA_DOCKER_RUN_ARGS,
        f"--user={os.getuid()}:{os.getgid()}",
        f"--env=DISPLAY={os.environ.get('DISPLAY', '')}",
        f"--volume={recordings_host_dir}:{CARLA_DOCKER_RECORDINGS_CONTAINER_DIR}",
        image,
        "bash", start_script_for(image), *CARLA_DOCKER_SERVER_ARGS, *server_args,
    ]


def start_carla(
        exe: str,
        *,
        render_off_screen: bool = True,
        quality_low: bool = True,
        map_name: str | None = None,
        boot_timeout: float = CARLA_BOOT_TIMEOUT_S,
        python_api: str | None = None,
        log: Callable[[str], None] | None = None,
) -> None:
    """
    Launch a CARLA server, wait until it is ready (see wait_until_ready) and
    optionally set the map. `exe` is either a local start script (see
    carla_versions.is_start_script) or the name of a docker image to start a
    fresh container from.
    """
    _log = log or print

    server_args = []
    if render_off_screen:
        server_args.append("-RenderOffScreen")
    if quality_low:
        server_args.append("-quality-level=Low")

    if is_start_script(exe):
        cmd = [exe, *server_args]
    else:
        # Make sure no stale container (e.g. left over from an unclean
        # previous kill) is still holding the name or the host network.
        subprocess.run(["docker", "rm", "-f", CARLA_DOCKER_CONTAINER_NAME],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        cmd = _docker_run_command(exe, server_args)

    _log(">> [CARLA] " + " ".join(cmd))
    server = subprocess.Popen(cmd)

    _log(">> [CARLA] Waiting for the server to boot")
    wait_until_ready(server, timeout=boot_timeout, python_api=python_api, log=_log)
    if map_name:
        ok = set_map(map_name, log=_log)
        if not ok:
            _log(f">> [CARLA] Warning: failed to set map '{map_name}'")


def restart_carla(
        exe: str,
        *,
        render_off_screen: bool = True,
        render_quality_low: bool = True,
        map_name: str | None = None,
        boot_timeout: float = CARLA_BOOT_TIMEOUT_S,
        python_api: str | None = None,
        log: Callable[[str], None] | None = None,
) -> None:
    """
    Hard-restart the CARLA server and wait until it is ready; optionally load a map.

    The routine is a convenience wrapper around:
    kill_carla -> wait_until_stopped -> start_carla (with flags/map, waits until ready).

    Parameters
    ----------
    exe : str
        CARLA start script or docker image, forwarded to ``start_carla``.
    render_off_screen : bool, default False
        Forwarded to ``start_carla``.
    render_quality_low : bool, default False
        Forwarded to ``start_carla``.
    map_name : str | None, default None
        Forwarded to ``start_carla`` to load after boot.
    boot_timeout : float
        Longest wait for the new server to become ready.
    python_api : str | None
        CARLA Python API version for the readiness check, see ``wait_until_ready``.
    log : Callable[[str], None] | None, default None
        Optional logger (defaults to ``print``).
    """
    _log = log or print

    _log(">> [CARLA] Killing existing instances")
    kill_carla(_log)
    wait_until_stopped(log=_log)

    _log(">> [CARLA] Starting new server")
    start_carla(
        exe,
        render_off_screen=render_off_screen,
        quality_low=render_quality_low,
        map_name=map_name,
        boot_timeout=boot_timeout,
        python_api=python_api,
        log=_log,
    )


def restart_and_connect(
        exe: str,
        host: str = CARLA_HOST,
        port: int = CARLA_PORT,
        timeout: float = 600,
        *,
        render_off_screen: bool = True,
        render_quality_low: bool = True,
        map_name: str | None = None,
        boot_timeout: float = CARLA_BOOT_TIMEOUT_S,
        log: Callable[[str], None] | None = None
):
    """
    Restart CARLA and return an active `carla.Client`. Only usable where the
    CARLA Python API is importable (worker subprocesses).

    Parameters
    ----------
    exe : str
        CARLA start script or docker image, passed through to `restart_carla`.
    host : str, default ``"localhost"``
        Connection endpoint for the CARLA RPC server.
    port: int, default ``2000``
        Connection endpoint port for the CARLA RPC server.
    timeout : float, default 600
        Seconds before a socket operation on the client aborts.
    render_off_screen, render_quality_low : bool
        Forwarded to `restart_carla`.
    boot_timeout : float
        Forwarded verbatim to `restart_carla`.
    log : Callable[[str], None], optional
        Logger callback (defaults to `print`).

    Returns
    -------
    carla.Client
        A connected client of Carla ready for world interaction.
    """
    import carla

    _log = log or print
    restart_carla(
        exe,
        render_off_screen=render_off_screen,
        render_quality_low=render_quality_low,
        map_name=map_name,
        boot_timeout=boot_timeout,
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
        host: str = CARLA_HOST,
        port: int = CARLA_PORT,
        log: Callable[[str], None] | None = None,
) -> bool:
    """
    Change the running map. Returns True on success, False otherwise.
    """
    import carla

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
