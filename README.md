# on-vehicle

This is the code that we actively run on the car

## Branches

### Stable

the branch named "stable" is the most recent branch that we have tested and ran on the car.

Other save points, such as "stable_lasvegas" contain stable branches from different points in time
new save point branches should be created with the form "stable_'branchdescriptor'"

### Dev

the branch named "dev" contains the main development thread where we test new things

other dev branches for specific features may be called "dev_'featurename'"

## qa_dev
this branch is for secondary quality testing after merging feature branches into dev. This code should be completely tested on simulation, and is the latest code that should be permitted to be run on actual hardware

## main
this branch is the most recent code that has been tested and verified on the actual hardware

# Install instructions: Docker (Recommended):
## Setup:
- install docker engine
- clone the repo
- **IMPORTANT** If you forgot to add `--recursive` option upon cloning, run `scripts/fix_git_submodules.sh` upon initial clone of this repository.
`scripts/fix_git_submodules.sh` is provided to backup messed-up submodule folders and pull them again.
- **IMPORTANT** Run `git submodule update --init --recursive` inside the repository directory when: *a)* Initial cloning; *b)* Checking out to a new branch; *c)* After each non recursive git pull.
Do `git pull --recurse-submodules` and `git clone --recursive` whenever you can to ensure the submodules are populated or updated properly.

## Running:
**IMPORTANT CHECK BEFORE YOU RUN:** In `compose.yml` lines 25-26 section `devices`: If running on the physical race car, make sure this section is *uncommented* ! Otherwise, comment out the `devices` section before running `docker compose up`.

```sh
export DOCKER_BUILDKIT=true
xhost +local:docker
docker compose up
```

*`xhost +local:docker` only works on linux. code may be run on other os without using xhost + but graphical elements may not work*

### Environment variables used for docker compose:

- `simulation`: Whether running with SVL simulator. Defaults to `false`.
- `debug`: Whether to output C# debug information to console. Defaults to `false`.
- `domain_id`: `ROS_DOMAIN_ID` to launch the container into. Defaults to system `${ROS_DOMAIN_ID}`.

The environment variables can be used together, e.g.:
```sh
simulation=true debug=true domain_id=11 docker compose up
```

## Clean build
Build cache and artifacts are stored and hidden inside the docker container
To build from scratch without these artifacts, you simply have to re-create the docker container from the on-vehicle image

To do this, run the following commands:
```sh
docker compose down
docker compose up
```

If you continue to run into issues, you can try re-building the image with
```sh
docker compose build
docker compose up
```

# Debug with ADE:
Compose the container (ADE variant) with:
```sh
DOCKER_BUILDKIT=true docker build -t bng_on_vehicle_ade:latest -f ./Dockerfiles/on_vehicle.dockerfile .
```
Then, at workspace folder, run:
```sh
ade start
ade enter
```
Once you are inside the ADE container, you are in a virtualized "native" environment, landed at `/home/$USER` being the workspace folder. Here, the dependent packages are all managed by the Docker, and the native build and native launch should work out-of-the-box. Please follow the steps below (Native build).

# Native build:
**This should only be used as a last resort on clean installations of Ubuntu if there are problems with docker**
Native build allows you to build and run the code outside of the docker container

*Do not edit anything in the native_build folder*

## Setup
```sh
./setup_scripts/master_setup.sh
```

## Build
Turn-a-key script to build everything:
```sh
./scripts/native_build.sh
```
Run from the root of this repo to build the code locally.

### Native build options:
- `-c`: Performs a clean build.
- `-d`: Outputs C# debug information to console.

## Launch
Turn-a-key script to launch everything:
```sh
./scripts/native_launch.sh
```
Run from the root of this repo to launch the code

### Environment variables used for native launch:

- `simulation`: Whether running with SVL simulator. Defaults to `false`.
- `domain_id`: `ROS_DOMAIN_ID` to launch the stack. Defaults to system `${ROS_DOMAIN_ID}`.

The environment variables can be used together, e.g.:
```sh
simulation=true domain_id=11 ./scripts/native_launch.sh
```
### Special native launches (for debugging):

- Sensor debugging: `ros2 launch iac_launch sensors.launch.py test_sensors_with_urdf:=true enable_cameras:=true enable_radars:=true`

- Vehicle and communication debugging: `ros2 launch iac_launch vehicle.launch.py`

### When NOT to launch:

- If `native_build.sh` is run within an ADE container and `native_launch.sh` is run on a physical computer, OR the other way around, the launch will NOT work. Running the `native_launch.sh` will throw an error directly. If you want to debug ROS2 messages of a workspace build in another environment, please see below (Native message-only build and environment setup).

# Native message-only build and environment setup:

A script is provided to build and source all project-related custom messages `*_msgs`. This script is useful for using `ros2 bag` and `ros2 topic` utilities when a native ROS version is installed but the build is launched within a container (or within another container). Run:
```sh
source ./scripts/native_msg_env.sh
```
to source all project-specific messages in the native ROS2 environment.

# Adding build dependencies 
When the dockerfile is updated with new dependencies, please update at the same time:
- `./setup_scripts/install_software.sh` for `apt`-installed software
- `py_requirements.txt` for `pip`-installed software
- `nuget_packages.config` for C# libraries pulled from NuGet.

*For the following situations, make sure you know what you are doing and test thoroughly if you have to add deps this way. Please test the consistency for BOTH docker and native execution.*
- `./setup_scripts/install_driver.sh` for third-party drivers that are compiled and installed from source.
- `./setup_scripts/misc_setup.sh` for all other software that are not presentd as a ROS 2 component.

### VS-Code Extensions
A list of useful extensions for syntax highlighting and auto-completion are provided in `vscode-extensions.list`. Install them *after* installing Visual Studio Code, using:
```sh
cat vscode-extensions.list |% { code --install-extension $_}
```

# Switching between tracks
There are some gps coords in state_estimation that need to be changed
Change the path parameter in path_server



# Projects used
#### Autoware auto
https://www.autoware.org/autoware-auto
https://gitlab.com/autowarefoundation/autoware.auto
https://gitlab.com/blackandgoldautonomousracing/AutowareAuto

#### IAC BVS
https://gitlab.com/IACBaseSoftware

#### ros2cs C# for ROS2
https://github.com/RobotecAI/ros2cs
https://github.com/BlackAndGoldAutonomousRacing/ros2cs
