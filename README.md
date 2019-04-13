# Assistance Arbitration

Feeling alone? Need help? Don't worry, we've got you covered. Our novel algorithms will help you choose what's the best help for you! Step right up.

Modules:

- [`assistance_msgs`](assistance_msgs/) - The primary interface between all the other modules in this folder.
- [`task_executor`](task_executor/) - Executes fetch and deliver tasks according to a program specification in YAML (now; perhaps CodeIt! in the future).
- [`assistance_arbitrator`](assistance_arbitrator/) - Research code that uses context passed on from the task_executor (or another node that might be requesting assistance), and decides where to forward the request - local or remote.
- [`local_interfaces`](local_interfaces/) - If we decide to solicit help from a local person, then the code in this folder (main entrypoint: [`local_interfaces/local_strategy`](local_interfaces/local_strategy)) handles the presentation of the request to a human.
- [`remote_interfaces`](remote_interfaces/) - If we decide to solicit help from a remote person, then the code in this folder (main entrypoint: [`remote_interfaces/remote_strategy`](remote_interfaces/remote_strategy)) handles the presentation of the request to a human.

Installation:

- [`install`](install/) - `rosinstall` files to setup and synchronize the different workspaces. Also includes `requirements.txt`.


## Prerequisites

The code in this repository requires the installation of the following prerequisites:

```yaml
# Python prerequisites. You can use the requirements.txt in `install/`
pip:
- dash
- joblib
- matplotlib
- networkx
- numpy
- psutil
- pydub
- plotly
- requests
- ruamel.yaml
- sklearn
- treeinterpreter
- wstool

# Recommended prerequisite for tts. Instructions are in the speech_interface package in this repository
docker:
- gtrail/marytts:dfki-prudence-hsmm

# (Optional) Prerequisite for tts. Instructions are in the speech_interface package in this repository
java:
- marytts

# Prerequisite packages. Ensure these are in your ROS workspace. You can use the rosinstall files in `install/`
catkin:
- GT-RAIL/fetch_demos
- GT-RAIL/fetch_gazebo
- GT-RAIL/fetch_grasp_suggestion
- GT-RAIL/fetch_ros
- HLP-R/hlpr_speech
- GT-RAIL/rail_agile
- GT-RAIL/rail_grasp_calculation
- GT-RAIL/rail_manipulation_msgs
- GT-RAIL/rail_object_detection_msgs
- GT-RAIL/rail_people_detection
- GT-RAIL/rail_segmentation
```


## Quickstart

The bulk of the code in this repository is written in service of the `task_executor` that executes a specified task. To that end, there is only a single launch file in the `task_executor` package that brings up both the dependencies of `task_executor`, as well as the executor itself:

1. Start the Fetch. In the real world, simply turn on the power; in simulation
```bash
roslaunch fetch_gazebo playground.launch
```
2. Start the MoveIt, move_base, grasp planners, speech, etc
```bash
# Use the sim flag only in simulation
roslaunch task_executor fetch_deliver.launch sim:=true start_all:=true task_executor:=false
```
3. Start the task_executor
```bash
# Use the sim flag only in simulation
roslaunch task_executor fetch_deliver.launch sim:=true task_executor:=true
```
4. Run a task defined in `$(find task_executor)/config/tasks.yaml`
```bash
# For example, to run the task with the key `main` in the YAML file
rosrun task_executor run_task.py main
```
