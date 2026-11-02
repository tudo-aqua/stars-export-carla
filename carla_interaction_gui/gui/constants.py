# Official CARLA maps from the docs (non-layered + layered "_Opt")
ALLOWED_NON_LAYERED_MAPS = [
    "Town01",
    "Town02",
    "Town03",
    "Town04",
    "Town05",
    "Town10HD",
]
ALLOWED_PARKED_MAPS = [
    "Town03",
    "Town05",
    "Town10HD",
]
ALLOWED_CARLA_MAPS = ALLOWED_NON_LAYERED_MAPS

# All drivable vehicle blueprints of CARLA 0.9.16 (cars, vans, trucks, buses,
# motorcycles, bicycles), selectable as the ego vehicle for manual driving.
# Keep "vehicle.lincoln.mkz_2017" first: it's the long-standing hardcoded
# default and must stay the fallback.
ALLOWED_EGO_VEHICLES = [
    "vehicle.lincoln.mkz_2017",
    # Cars
    "vehicle.lincoln.mkz_2020",
    "vehicle.audi.a2",
    "vehicle.audi.tt",
    "vehicle.audi.etron",
    "vehicle.bmw.grandtourer",
    "vehicle.chevrolet.impala",
    "vehicle.citroen.c3",
    "vehicle.dodge.charger_2020",
    "vehicle.dodge.charger_police",
    "vehicle.dodge.charger_police_2020",
    "vehicle.ford.crown",
    "vehicle.ford.mustang",
    "vehicle.jeep.wrangler_rubicon",
    "vehicle.mercedes.coupe",
    "vehicle.mercedes.coupe_2020",
    "vehicle.micro.microlino",
    "vehicle.mini.cooper_s",
    "vehicle.mini.cooper_s_2021",
    "vehicle.nissan.micra",
    "vehicle.nissan.patrol",
    "vehicle.nissan.patrol_2021",
    "vehicle.seat.leon",
    "vehicle.tesla.model3",
    "vehicle.toyota.prius",
    # Vans
    "vehicle.ford.ambulance",
    "vehicle.mercedes.sprinter",
    "vehicle.volkswagen.t2",
    "vehicle.volkswagen.t2_2021",
    # Trucks
    "vehicle.carlamotors.carlacola",
    "vehicle.carlamotors.european_hgv",
    "vehicle.carlamotors.firetruck",
    "vehicle.tesla.cybertruck",
    # Buses
    "vehicle.mitsubishi.fusorosa",
    # Motorcycles
    "vehicle.harley-davidson.low_rider",
    "vehicle.kawasaki.ninja",
    "vehicle.vespa.zx125",
    "vehicle.yamaha.yzf",
    # Bicycles
    "vehicle.bh.crossbike",
    "vehicle.diamondback.century",
    "vehicle.gazelle.omafiets",
]

# All drivable vehicle blueprints of CARLA 0.10.0 (UE5 ships far fewer, under
# different ids). The first one is the fallback.
ALLOWED_EGO_VEHICLES_0_10 = [
    "vehicle.lincoln.mkz",
    "vehicle.dodge.charger",
    "vehicle.dodgecop.charger",
    "vehicle.mini.cooper",
    "vehicle.nissan.patrol",
    "vehicle.taxi.ford",
    "vehicle.ambulance.ford",
    "vehicle.sprinter.mercedes",
    "vehicle.carlacola.actors",
    "vehicle.firetruck.actors",
    "vehicle.fuso.mitsubishi",
]

# "Add controlled actor" buttons of the Manual Drive tab: label -> blueprint
# filter (a wildcard filter spawns a random match).
CONTROLLED_ACTORS = {
    "Cyclist": "vehicle.bh.crossbike",
    "Walker": "walker.pedestrian.*",
    "Small car": "vehicle.mini.cooper_s_2021",
    "Truck": "vehicle.carlamotors.carlacola",
}
CONTROLLED_ACTORS_0_10 = {
    "Walker": "walker.pedestrian.*",
    "Small car": "vehicle.mini.cooper",
    "Truck": "vehicle.carlacola.actors",
}
# Used when the CARLA version is unknown: filters matching in every version.
ALL_CONTROLLED_ACTORS = {
    "Cyclist": "vehicle.bh.crossbike",
    "Walker": "walker.pedestrian.*",
    "Small car": "vehicle.mini.cooper*",
    "Truck": "vehicle.*carlacola*",
}

# CARLA docker images offered in the "CARLA executable" dropdown. Per image:
#   python_api      - CARLA version whose Python API (libcarla wheel) matches
#                     the server; selecting the image auto-selects it.
#   start_script    - server start script inside the container's working dir
#                     (CarlaUE4.sh for UE4 builds, CarlaUnreal.sh for UE5).
#   python_api_dir  - folder inside the image holding the PythonAPI wheels;
#                     the matching wheel is copied out of it on first use
#                     (or downloaded from PyPI if the image has none for
#                     this Python version).
#   maps            - maps shipped with that image, offered in the map lists.
#   vehicles        - drivable vehicle blueprints, offered in the vehicle lists.
#   controlled_actors - "Add controlled actor" buttons, see CONTROLLED_ACTORS.
CARLA_DOCKER_VERSIONS = {
    "carlasim/carla:0.9.15": {
        "python_api": "0.9.15",
        "start_script": "CarlaUE4.sh",
        # Ships wheels for Python 2.7/3.7 only; ensure_python_api falls back to PyPI.
        "python_api_dir": "/home/carla/PythonAPI/carla/dist",
        "maps": ALLOWED_NON_LAYERED_MAPS,
        "vehicles": ALLOWED_EGO_VEHICLES,
        "controlled_actors": CONTROLLED_ACTORS,
    },
    "carlasim/carla:0.9.16": {
        "python_api": "0.9.16",
        "start_script": "CarlaUE4.sh",
        "python_api_dir": "/workspace/PythonAPI/carla/dist",
        "maps": ALLOWED_NON_LAYERED_MAPS,
        "vehicles": ALLOWED_EGO_VEHICLES,
        "controlled_actors": CONTROLLED_ACTORS,
    },
    "carlasim/carla:0.10.0": {
        "python_api": "0.10.0",
        "start_script": "CarlaUnreal.sh",
        "python_api_dir": "/home/carla/PythonAPI/carla/dist",
        "maps": ["Town10HD_Opt", "Mine_01"],
        "vehicles": ALLOWED_EGO_VEHICLES_0_10,
        "controlled_actors": CONTROLLED_ACTORS_0_10,
    },
}

# Preselected when no (existing) CARLA executable is configured.
CARLA_DEFAULT_DOCKER_IMAGE = "carlasim/carla:0.9.16"

# Every map/vehicle of every catalogued version, offered when the CARLA executable is
# a free-text docker image or a shell script whose version is unknown.
ALL_CARLA_MAPS = list(dict.fromkeys(m for v in CARLA_DOCKER_VERSIONS.values() for m in v["maps"]))
ALL_CARLA_VEHICLES = list(dict.fromkeys(m for v in CARLA_DOCKER_VERSIONS.values() for m in v["vehicles"]))

# Start script used for docker images that aren't in CARLA_DOCKER_VERSIONS:
# picked by the image tag, CARLA_UE5_START_SCRIPT from CARLA_UE5_MIN_VERSION on.
CARLA_UE4_START_SCRIPT = "CarlaUE4.sh"
CARLA_UE5_START_SCRIPT = "CarlaUnreal.sh"
CARLA_UE5_MIN_VERSION = (0, 10)

# File extensions that mark the CARLA executable as a local start script;
# anything else is treated as a docker image name.
CARLA_SCRIPT_EXTENSIONS = (".sh",)

# docker run settings for launching CARLA from an image. The container name
# is also what kill_carla() stops.
CARLA_DOCKER_CONTAINER_NAME = "carla_server"
CARLA_DOCKER_RUN_ARGS = [
    "--rm",
    "--runtime=nvidia",
    "--net=host",
    "--env=NVIDIA_VISIBLE_DEVICES=all",
    "--env=NVIDIA_DRIVER_CAPABILITIES=all",
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw",
]
CARLA_DOCKER_SERVER_ARGS = ["-nosound"]
# Host folder mounted into the container for recordings (host, container side).
CARLA_DOCKER_RECORDINGS_HOST_DIR = "~/workspace/carlarecordings"
CARLA_DOCKER_RECORDINGS_CONTAINER_DIR = "/workspace/recordings"

# Where the Python APIs copied out of the docker images are cached, one
# subfolder per CARLA version.
CARLA_PYTHON_API_CACHE_DIR = "~/.cache/stars-export-carla/carla-python-api"
