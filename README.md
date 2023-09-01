# Carla simulation-runs generator

This repository consists of a handful of useful function to automatically extract simulation-runs from the
[Carla Simulator](https://carla.org/) which then can be used by
the [CARLA Importer](https://github.com/tudo-aqua/stars/tree/main/stars-import-carla)
of the [STARS framework](https://github.com/tudo-aqua/stars). Examples of the resulting data can be
found [here](https://zenodo.org/record/8131947).

## Setup

### Python

<details>

  <summary>Linux/Mac</summary>

### Python 3.7

To use the functions of this repository you need Python 3.7. Follow the instructions on
the [official website](https://www.python.org/downloads/release/python-370/)
to install Python 3.7 on your system.

### Virtual Environment

This repository requires a virtual environment. Follow these instructions to initialize a new virtual environment.

Install "virtualenv"

1. ``python3 -m pip install --user --upgrade pip``
2. ``python3 -m pip install --user virtualenv``

Now navigate to the root folder of this repository.

3. ``cd your/local/folder/stars-export-carla``

Create a virtual environment.

4. ``python3 -m venv venv``

Your virtual environment is now setup.

### Install Requirements

This repository depends on specific libraries to be correctly loaded into the virtual environment.
To install these requirements follow these instructions:

Now navigate to the root folder of this repository.

1. ``cd your/local/folder/stars-export-carla``

Install the ``requirements.txt``

2. ``./venv/Scripts/python -m pip install -r ./requirements.txt``

This will install all necessary requirements.

</details>

<details>

  <summary>Windows</summary>

### Python 3.7

To use the functions of this repository you need Python 3.7. Follow the instructions on
the [official website](https://www.python.org/downloads/release/python-370/)
to install Python 3.7 on your system.

### Virtual Environment

This repository requires a virtual environment. Follow these instructions to initialize a new virtual environment.

Install "virtualenv"

1. ``py -m pip install --user --upgrade pip``
2. ``py -m pip install --user virtualenv``

Now navigate to the root folder of this repository.

3. ``cd your/local/folder/stars-export-carla``

Create a virtual environment.

4. ``py -m venv venv``

Your virtual environment is now setup.

### Install Requirements

This repository depends on specific libraries to be correctly loaded into the virtual environment.
To install these requirements follow these instructions:

Now navigate to the root folder of this repository.

1. ``cd your/local/folder/stars-export-carla``

Install the ``requirements.txt``

2. ``./venv/Scripts/python.exe -m pip install -r ./requirements.txt``

This will install all necessary requirements.

</details>

### Carla

Firstly, you have to [install Carla](https://github.com/carla-simulator/carla/releases/tag/0.9.14). Currently, this
repository
supports Carla 0.9.14

Now update the project with the path to your local Carla installation in:
<details><summary>Linux/Mac</summary>

`scripts/shell_scripts/config.sh`
</details>
<details><summary>Windows</summary>

`scripts/batch_scripts/config.bat`
</details>

## Generation data types

The functions in this repository generate three kinds of data types:

1. **Map**: In here, all "static" datapoints of the underlying road/lane structure for one specific map is
   stored
   <details><summary>Class details</summary>
    - DataBlock
    - DataRoad
    - DataLane
    - DataLaneMidpoint
    - DataSpeedLimit
    - DataContactArea
    - DataLandmark
    - DataStaticTrafficLight
    - DataContactLaneInfo
   </details>
2. **Dynamic**: In here, all "dynamic" datapoints, such as Vehicles, Pedestrians, and the traffic light states are
   stored
   <details><summary>Class details</summary>
    - TickData
    - DataActorPosition
    - DataActor
    - DataTrafficLight
    - DataPedestrian
    - DataTrafficSign
    - DataVehicle
   </details>
3. **Weather**: In here, the weather parameters of a scenario is stored
   <details><summary>Class details</summary>
    - DataWeatherParameters
   </details>

## Generation Scripts

There are pre-defined scripts that automate the process of generating scenario data files.

- ``generate_data``: Generates recordings, and then monitors them and generates map data
- ``generate_map_data``: Only generate map data for all currently supported Carla maps
- ``generate_recording``: Only generate one recording with a set seed
- ``generate_recordings``: Only generate the set number of seeds (0-100)
- ``generate_video``: Generate a video for a given seed, an actor id and the start and end time
- ``monitor_all_recordings``: Iterate through all generated recordings and monitor all of them
- ``monitor_recording``: Monitor one specific recording (by providing a seed)

## Generation Order

This repository follows a specific order of generation steps, which are summarized here:

1. Generation of recordings (`scripts/generate_recording`)
2. Generation of weather parameters
3. Generation of dynamic data (`scripts/monitor_recording`)

Each step requires the results of the step above. Next, the generation steps are discussed in more detail.
Every generated file is located in the ``generated-data`` folder, which is created during the generation.

Besides the two generation targets, the map data can be generated separately.

3. Generation of map data (`scripts/generate_map_data`)

### Generation of Recordings

It is possible to call Carla's simulation with a set seed, so that the actor spawns and the route planning of the
autonomous
agents will always behave the same. Nevertheless, do the results differ in exact positions and timings of the actors in
the
simulation. Therefore, as a first step a [Recording](https://carla.readthedocs.io/en/latest/adv_recorder/#recording) of
the
simulation is created. This stores the exact states of each actor (e.g. Vehicles, Pedestrian, Trafficlight states,
etc.).

Each recording is generated with a specific seed, so that the generated files will be generated into:
``/recordings/{map_name}/{map_name}_seed{seed}.zip``.
With these recordings it is possible to recreate specific scenario 1:1.

Each recoding generates random weather parameters that are set for the simulation.

### Generation of Weather Parameters

For each recording a random set of weather parameters is generated.

Each Recording generates exactly one set of weather parameters which is generated into:
``/recordings/{map_name}/weather_data_{map_name}_seed{seed}.zip``. With the seed in the generated file name,
each recording can be mapped to its weather parameters.

### Generation of Dynamic Data

Each recording is then replayed and monitored. The monitor tracks the orientation and position of each actor in the
whole
simulation at all times. The dynamic data also includes the weather parameters.

For each recording file and the corresponding weather parameters file, a single dynamic data file is generated into:
``/simulations_runs/{map_name}/dynamic_data_{map_name}_seed{seed}.zip``.

### Generation of Map Data

For each map, the _static_ information is extracted and stored in
the **map** data type. For each map there is only one file generated, as the static data is persistent between
scenarios.
The map data will be generated into:  ``/simulation_runs/{map_name}/static_data_{mapName}.zip``

Currently supported:

- Town01
- Town02
- Town10