#!/usr/bin/env python
# Randomize the test environment and then launch

from __future__ import print_function, division

import os
import sys
import shlex
import argparse
import subprocess

import numpy as np

import rospkg

# Import isolation
try:
    import isolation
except ImportError as e:
    sys.path.append('/home/banerjs/Libraries/RAIL/codebase/banerjs/isolation/models')
    import isolation
from isolation.data.annotations import Annotations


# The bulk of the script. TODO: Make this a bit smarter

AVAILABLE_ROSLAUNCH_COMMANDS = [
    # Base stalled
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=true base_collided:=false incorrect_map:=false hardware_failure:=false gui:=false",

    # Base moved back
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=true base_stalled:=false base_collided:=false incorrect_map:=false hardware_failure:=false gui:=false",

    # Incorrect localization
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=false base_collided:=false incorrect_map:=true hardware_failure:=false gui:=false",

    # Base collided
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=false base_collided:=true incorrect_map:=false hardware_failure:=false gui:=false"
]

ROSLAUNCH_TEMPLATE = \
"""
roslaunch fetch_gazebo task_worlds.launch
object_location_idx:={object_location_idx}
pickup_distraction:={pickup_distraction}
place_distraction:={place_distraction}
door_blocked:={door_blocked}
door_block_invisible:={door_block_invisible}
head_moved:={head_moved}
base_moved_back:={base_moved_back}
base_stalled:={base_stalled}
base_collided:={base_collided}
incorrect_map:={incorrect_map}
hardware_failure:={hardware_failure}
gui:=false
initial_pose_x:={initial_pose_x}
initial_pose_y:={initial_pose_y}
initial_pose_a:={initial_pose_a}
"""

ROSLAUNCH_WORLDS = os.path.join(
    rospkg.RosPack().get_path('fetch_gazebo'),
    "worlds",
    "{object_name}_{object_location_idx}_{pickup_distraction}_{place_distraction}_{door_blocked}_{door_block_invisible}_{base_stalled}_zone.sdf"
)


def _create_no_fault_worlds():
    no_fault_worlds = set()

    for name_option in Annotations.OBJECT_NAME_OPTIONS:
        for location_option in Annotations.OBJECT_LOCATION_IDX_OPTIONS:

            # Create the corresponding world. We assume that the first option
            # among the fault injection options is a no-fault option
            world_options = {}
            for option, attr in Annotations.ANNOTATIONS_ATTR_MAP.iteritems():
                attr = getattr(Annotations, attr)

                if attr == Annotations.OBJECT_NAME_OPTIONS:
                    world_options[option] = name_option['value']
                elif attr == Annotations.OBJECT_LOCATION_IDX_OPTIONS:
                    world_options[option] = location_option['value']
                else:
                    world_options[option] = str(attr[0]['value']).lower()

            # Check that such a world exists. If so, add it
            world_path = ROSLAUNCH_WORLDS.format(**world_options)
            if os.path.exists(world_path):
                no_fault_worlds.add(world_path)

    return no_fault_worlds


def _check_valid_options(run_options, no_fault_worlds, allow_fault_free):
    pass


def main(randomization_options, allow_fault_free, debug):
    # First calculate the world configurations that have no faults
    no_fault_worlds = _create_no_fault_worlds()
    if debug:
        print("Calculated no fault worlds:")
        for world_path in no_fault_worlds:
            print(world_path)

    # Then iterate until we have a run config that matches our desired
    run_options = {}
    for option_name, attr_name in Annotations.ANNOTATIONS_ATTR_MAP.iteritems():
        # if option_name in set([object_location_idx]):
        pass

    if debug:
        print("Simulation will have the following options:")
        for option_name, value in run_options.iteritems():
            print("\t", option_name, value)

    # process = subprocess.check_call(np.random.choice(AVAILABLE_ROSLAUNCH_COMMANDS), shell=True)
    # return process


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-d', '--debug', action='store_true',
                            help="Run this script in debug mode")
    arg_parser.add_argument('--randomize_locations', action='store_true',
                            help="Randomize the location of the objects")
    arg_parser.add_argument('--randomize_movement', action='store_true',
                            help="Randomize the movement faults")
    arg_parser.add_argument('--randomize_perception', action='store_true',
                            help="Randomize the perception faults")
    arg_parser.add_argument('--randomize_beliefs', action='store_true',
                            help="Randomize the belief faults")
    arg_parser.add_argument('--no_fault_free', action='store_true',
                            help="Flag indicating that at least one fault must be injected")
    args = arg_parser.parse_args()
    randomization_options = {
        "randomize_locations": args.randomize_locations,
        "randomize_movement": args.randomize_movement,
        "randomize_beliefs": args.randomize_beliefs,
        "randomize_perception": args.randomize_perception,
    }

    main(randomization_options, not args.no_fault_free, args.debug)
