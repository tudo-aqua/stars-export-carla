from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Callable

from carla_interaction_gui.gui.constants import (
    ALL_CARLA_MAPS,
    ALL_CARLA_VEHICLES,
    ALL_CONTROLLED_ACTORS,
    CARLA_DOCKER_VERSIONS,
    CARLA_PYTHON_API_CACHE_DIR,
    CARLA_SCRIPT_EXTENSIONS,
    CARLA_UE4_START_SCRIPT,
    CARLA_UE5_MIN_VERSION,
    CARLA_UE5_START_SCRIPT,
)

# Loose docker image reference: [registry[:port]/]name[/name...][:tag]
_IMAGE_REFERENCE = re.compile(r"^[a-z0-9]+([._-][a-z0-9]+)*(:[0-9]+)?(/[a-z0-9]+([._-][a-z0-9]+)*)*(:[\w][\w.-]*)?$")


def is_start_script(target: str) -> bool:
    """Whether the CARLA executable is a local start script (by extension) rather than a docker image."""
    return target.strip().lower().endswith(CARLA_SCRIPT_EXTENSIONS)


def python_api_versions() -> list[str]:
    """All CARLA versions whose Python API can be extracted from a catalogued image."""
    return list(dict.fromkeys(v["python_api"] for v in CARLA_DOCKER_VERSIONS.values()))


def python_api_for(target: str) -> str | None:
    """The Python API version belonging to a catalogued image, None for anything else."""
    entry = CARLA_DOCKER_VERSIONS.get(target.strip())
    return entry["python_api"] if entry else None


def maps_for(target: str) -> list[str]:
    """Maps shipped with a catalogued image; every known map for free-text images and scripts."""
    entry = CARLA_DOCKER_VERSIONS.get(target.strip())
    return list(entry["maps"]) if entry else list(ALL_CARLA_MAPS)


def vehicles_for(target: str) -> list[str]:
    """Vehicle blueprints of a catalogued image; every known one for free-text images and scripts."""
    entry = CARLA_DOCKER_VERSIONS.get(target.strip())
    return list(entry["vehicles"]) if entry else list(ALL_CARLA_VEHICLES)


def controlled_actors_for(target: str) -> dict[str, str]:
    """'Add controlled actor' buttons (label -> blueprint filter) for the given CARLA executable."""
    entry = CARLA_DOCKER_VERSIONS.get(target.strip())
    return dict(entry["controlled_actors"]) if entry else dict(ALL_CONTROLLED_ACTORS)


def start_script_for(image: str) -> str:
    """
    The server start script inside the container: from the catalogue if the
    image is listed, otherwise chosen by the version in the image tag.
    """
    entry = CARLA_DOCKER_VERSIONS.get(image)
    if entry:
        return entry["start_script"]
    match = re.search(r":(\d+)\.(\d+)", image)
    if match and (int(match.group(1)), int(match.group(2))) >= CARLA_UE5_MIN_VERSION:
        return CARLA_UE5_START_SCRIPT
    return CARLA_UE4_START_SCRIPT


def _docker_image_exists(image: str) -> bool:
    try:
        result = subprocess.run(["docker", "image", "inspect", image],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def validate_target(target: str) -> str | None:
    """Check the CARLA executable; returns an error message, or None if it can be launched."""
    target = target.strip()
    if not target:
        return "CARLA executable required."
    if is_start_script(target):
        if not os.path.isfile(target):
            return f"CARLA start script does not exist:\n{target}"
        return None
    if not _IMAGE_REFERENCE.match(target):
        return f"CARLA executable is neither a start script ({', '.join(CARLA_SCRIPT_EXTENSIONS)}) " \
               f"nor a valid docker image name:\n{target}"
    if not _docker_image_exists(target):
        return f"Docker image not found locally (pull it first with 'docker pull {target}'):\n{target}"
    return None


def _python_api_cache_dir(version: str) -> Path:
    return Path(CARLA_PYTHON_API_CACHE_DIR).expanduser() / version


def _is_cached(version: str) -> bool:
    # The cache folder only appears once fully unpacked (see ensure_python_api),
    # and a wheel's dist-info is present whether it ships a package (0.9.x) or
    # a single extension module (0.10.x).
    return any(_python_api_cache_dir(version).glob("carla-*.dist-info"))


def validate_python_api(version: str) -> str | None:
    """Check the selected Python API version; returns an error message, or None if it is usable."""
    if not version:
        return "CARLA Python API required."
    if _is_cached(version) or version in python_api_versions():
        # Not cached yet: ensure_python_api fetches it from a local image or PyPI.
        return None
    return f"No docker image in the catalogue provides CARLA Python API {version}."


def _wheel_from_image(image: str, python_tag: str, dest: Path) -> Path | None:
    """Copy the image's PythonAPI wheels into `dest`; returns the one for `python_tag`, if any."""
    container = subprocess.run(["docker", "create", image], capture_output=True, text=True,
                               check=True).stdout.strip()
    try:
        subprocess.run(["docker", "cp", f"{container}:{CARLA_DOCKER_VERSIONS[image]['python_api_dir']}/.",
                        str(dest)], capture_output=True, check=True)
    finally:
        subprocess.run(["docker", "rm", container], capture_output=True)
    wheels = sorted(dest.glob(f"carla-*-{python_tag}-*.whl"))
    return wheels[0] if wheels else None


def _wheel_from_pypi(version: str, dest: Path) -> Path | None:
    """Download the carla wheel of `version` for this interpreter from PyPI into `dest`, if one exists."""
    result = subprocess.run([sys.executable, "-m", "pip", "download", f"carla=={version}", "--no-deps",
                             "--only-binary=:all:", "--dest", str(dest)], capture_output=True, text=True)
    wheels = sorted(dest.glob("carla-*.whl"))
    return wheels[0] if result.returncode == 0 and wheels else None


def ensure_python_api(version: str, log: Callable[[str], None] | None = None) -> Path:
    """
    Make the CARLA Python API of the given version importable and return the
    folder to put on PYTHONPATH. On first use, the wheel matching this
    interpreter is copied out of a local catalogued docker image or, if no
    image provides one (e.g. images shipping only wheels for older Pythons),
    downloaded from PyPI, and unpacked into the cache; later calls reuse it.
    """
    _log = log or print
    target = _python_api_cache_dir(version)
    if _is_cached(version):
        return target

    python_tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    images = [image for image, v in CARLA_DOCKER_VERSIONS.items()
              if v["python_api"] == version and _docker_image_exists(image)]
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=target.parent) as tmp:
        wheel_path = None
        for image in images:
            _log(f">> [CARLA] Copying Python API {version} ({python_tag}) out of '{image}'")
            wheel_path = _wheel_from_image(image, python_tag, Path(tmp, image.replace("/", "_")))
            if wheel_path:
                break
            _log(f">> [CARLA] '{image}' has no Python API wheel for {python_tag}")
        if not wheel_path:
            _log(f">> [CARLA] Downloading Python API {version} ({python_tag}) from PyPI")
            wheel_path = _wheel_from_pypi(version, Path(tmp, "pypi"))
        if not wheel_path:
            raise RuntimeError(f"Neither a local docker image nor PyPI provides a CARLA Python API {version} "
                               f"wheel for {python_tag}")

        unpacked = Path(tmp, "api")
        with zipfile.ZipFile(wheel_path) as wheel:
            wheel.extractall(unpacked)
        # Replace a broken leftover (e.g. from an interrupted copy) with the fresh one.
        shutil.rmtree(target, ignore_errors=True)
        unpacked.rename(target)

    _log(f">> [CARLA] Cached Python API {version} in {target}")
    return target


def python_api_env(version: str, log: Callable[[str], None] | None = None) -> dict[str, str]:
    """A copy of os.environ under which subprocesses import the CARLA Python API of the given version."""
    env = os.environ.copy()
    api_dir = str(ensure_python_api(version, log))
    env["PYTHONPATH"] = os.pathsep.join(p for p in (api_dir, env.get("PYTHONPATH", "")) if p)
    return env
