# Assistance Arbitrator

Given an incoming assistance goal, the arbitrator uses a method selected through config files to decide how to resolve the request. When it picks a method of resolution, called a **strategy**, it then forwards the request on to the appropriate strategy.

There are two strategies:

1. [Local Strategy](local_interfaces/local_strategy/): The robot looks for a local human, approaches them, and tries to solicit help from this human.
1. [Remote Strategy](remote_interfaces/remote_strategy): The robot sends a request to a remote interface, which is then presented to a user on that interface.


## Notes

- Probably should have used the diagnostics topic from the start to keep a trace of the program execution. Too late to start now.
- Instead, make sure to bag the trace topic and the diagnostics topic; we'll use both to create a complete trace of execution post-facto.

### Data Collection

When running data collection in simulation, run (from the Workspace):

```bash
# Start the simulation world
roslaunch fetch_gazebo task_worlds.launch \
    object_location_idx:=0|1|2 \
    pickup_distraction:=none|cluttered|empty \
    place_distraction:=none|cluttered \
    door_blocked:=false|true \
    door_block_invisible:=false|true \
    head_moved:=false|true \
    base_moved_back:=false|true \
    base_stalled:=false|true \
    base_collided:=false|true \
    incorrect_map:=false|true \
    hardware_failure:=false|true

# Start the task executor dependencies
roslaunch task_executor fetch_deliver.launch \
    sim:=true \
    start_all:=true \
    task_executor:=false \
    datalogger:=true \
    incorrect_map:=false|true

# Start debug aids, such as RViz
rosrun rviz rviz -d fetch.rviz
python visualize_run.py

# Start the logging services
rosservice call /datalogger/start
# Hit the visualizer start live update button

# Finally start the task executor
roslaunch task_executor fetch_deliver.launch sim:=true task_executor:=true local_strategy:=false

# And then execute a task
rosrun task_executor run_task.py <task_def>
```
