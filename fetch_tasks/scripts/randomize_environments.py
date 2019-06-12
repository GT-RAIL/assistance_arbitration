#!/usr/bin/env python
# Randomize the test environment and then launch

from __future__ import print_function, division

import os
import sys
import shlex
import argparse
import subprocess
import collections

import numpy as np

import rospkg

# Import isolation
try:
    import isolation
except ImportError as e:
    sys.path.append('/home/banerjs/Libraries/RAIL/codebase/banerjs/isolation/models')
    import isolation
from isolation.data.annotations import Annotations


# The bulk of the script.

DEBUG_ROSLAUNCH_COMMANDS = [
    # Base stalled
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=true base_collided:=false incorrect_map:=false hardware_failure:=false gui:=false",

    # Base moved back
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=true base_stalled:=false base_collided:=false incorrect_map:=false hardware_failure:=false gui:=false",

    # Incorrect localization
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=false base_collided:=false incorrect_map:=true hardware_failure:=false gui:=false",

    # Base collided
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=false base_collided:=true incorrect_map:=false hardware_failure:=false gui:=false"
]


# TODO: When we switch to melodic, remove the place and pickup helper args
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
gui:={debug}
initial_pose_x:={initial_pose_x}
initial_pose_y:={initial_pose_y}
initial_pose_a:={initial_pose_a}
todo_pickup_none:={todo_pickup_none}
todo_pickup_cluttered:={todo_pickup_cluttered}
todo_pickup_empty:={todo_pickup_empty}
todo_place_none:={todo_place_none}
todo_place_cluttered:={todo_place_cluttered}
"""


def _create_no_fault_worlds():
    # Create options that have no faults injected in them
    no_fault_worlds = []

    for name_option in Annotations.OBJECT_NAME_OPTIONS:
        for location_option in Annotations.OBJECT_LOCATION_IDX_OPTIONS:
            for x_option in Annotations.INITIAL_POSE_X_OPTIONS:
                for y_option in Annotations.INITIAL_POSE_Y_OPTIONS:
                    for a_option in Annotations.INITIAL_POSE_A_OPTIONS:
                        # Create the corresponding world. We assume that the
                        # first option among the fault injection options is a
                        # no-fault option
                        world_options = {}
                        for option, attr in Annotations.ANNOTATIONS_ATTR_MAP.iteritems():
                            if option in set(['task', 'result_annotation', 'gazebo_issue']):
                                continue

                            attr = getattr(Annotations, attr)

                            if attr == Annotations.OBJECT_NAME_OPTIONS:
                                world_options[option] = name_option['value']
                            elif attr == Annotations.OBJECT_LOCATION_IDX_OPTIONS:
                                world_options[option] = location_option['value']
                            elif attr == Annotations.INITIAL_POSE_X_OPTIONS:
                                world_options[option] = x_option['value']
                            elif attr == Annotations.INITIAL_POSE_Y_OPTIONS:
                                world_options[option] = y_option['value']
                            elif attr == Annotations.INITIAL_POSE_A_OPTIONS:
                                world_options[option] = a_option['value']
                            else:
                                world_options[option] = attr[0]['value']

                        if _check_options_valid(world_options):
                            no_fault_worlds.append(world_options)

    return no_fault_worlds


def _check_options_valid(run_options):
    # Check if the configuration is a valid configuration
    if run_options['initial_pose_x'] == 0 and run_options['initial_pose_y'] != 0:
        return False

    # TODO: With Melodic and parameterized launch files, we can remove this
    if run_options['base_stalled'] and not (run_options['initial_pose_x'] == 0
                                            and run_options['initial_pose_y'] == 0
                                            and run_options['initial_pose_a'] == 0):
        return False

    for option in Annotations.ANNOTATIONS_ATTR_MAP.iterkeys():
        if option in set(['task', 'result_annotation', 'gazebo_issue']):
            continue

        if option not in run_options:
            return False

    return True


def _check_run_valid(run_options, allow_fault_free, single_fault, no_fault_worlds, all_fault_options):
    # Check if the run configuration satisfies the command line args

    # Check if the run is empty
    if len(run_options) == 0:
        return False

    # If the configuration is invalid, then no need for further checks
    if not _check_options_valid(run_options):
        return False

    # Check if this is a no fault run and we want only faulty runs
    if not allow_fault_free and run_options in no_fault_worlds:
        return False

    # If this is meant to be a single fault run, then don't allow multiple fault
    # injections
    if single_fault:
        num_faults = 0
        for option, value in run_options.iteritems():
            # Check to see if the fault configuration is set to faulty
            attr = getattr(Annotations, Annotations.ANNOTATIONS_ATTR_MAP[option])
            if option in all_fault_options and value != Annotations.get_values(attr)[0]:
                num_faults += 1

            if num_faults > 1:
                return False

    # This is a valid configuration
    return True


def _generate_command(run_options):
    # Generate the command to run

    # TODO: Remove this in Melodic.
    # First add in the auxilliary pickup and place args
    run_options['todo_pickup_none'] = run_options['pickup_distraction'] == 'none'
    run_options['todo_pickup_empty'] = run_options['pickup_distraction'] == 'empty'
    run_options['todo_pickup_cluttered'] = run_options['pickup_distraction'] == 'cluttered'
    run_options['todo_place_none'] = run_options['place_distraction'] == 'none'
    run_options['todo_place_cluttered'] = run_options['place_distraction'] == 'cluttered'

    # Then convert all the options to lowercase strings
    for option in run_options.keys():
        run_options[option] = str(run_options[option]).lower()

    # Then get the command
    command = ROSLAUNCH_TEMPLATE.format(**run_options)
    return shlex.split(command)


def main(injection_options, allow_fault_free, single_fault, debug):
    # Associate the options in the launch file to the instructions in the
    # command line configs
    injection_sets = collections.OrderedDict([
        ('randomize_locations', set(['initial_pose_x', 'initial_pose_y', 'initial_pose_a'])),
        ('randomize_objects', set(['object_location_idx'])),
        ('inject_movement', set(['base_moved_back', 'base_stalled', 'base_collided', 'incorrect_map'])),
        ('inject_beliefs', set(['door_blocked', 'door_block_invisible'])),
        ('inject_perception', set(['pickup_distraction', 'place_distraction', 'head_moved', 'hardware_failure'])),
    ])
    all_fault_options = (
        injection_sets['inject_movement']
        | injection_sets['inject_beliefs']
        | injection_sets['inject_perception']
    )

    # First calculate the world configurations that have no faults
    no_fault_worlds = _create_no_fault_worlds()
    if debug:
        print("Calculated no fault worlds")

    # Then iterate until we have a run config that matches our desired
    run_options = {}
    while not _check_run_valid(run_options, allow_fault_free, single_fault, no_fault_worlds, all_fault_options):
        for option, attr in Annotations.ANNOTATIONS_ATTR_MAP.iteritems():
            attr = getattr(Annotations, attr)

            # Ignore the task and result keys
            if option in set(['task', 'result_annotation', 'gazebo_issue']):
                continue

            # For each of the randomization sets, inject the fault if the option
            # is set; else pick the no-fault (assumed 1st) option
            for injection_option_name, injection_set in injection_sets.iteritems():

                # The last injection set containing the option is overrides all
                # other previous options
                if option in injection_set and injection_options[injection_option_name]:
                    run_options[option] = np.random.choice(attr)['value']
                elif option in injection_set:
                    run_options[option] = attr[0]['value']

            # Finally, choose randomly among the options that are not part of
            # the injection sets
            if option not in run_options:
                run_options[option] = np.random.choice(attr)['value']

    if debug:
        print("Simulation will have the following options:")
        for option, value in run_options.iteritems():
            print("\t", option, value)

    run_options['debug'] = debug
    command = _generate_command(run_options)
    if debug:
        print("Command:", command)

    # process = subprocess.check_call(shlex.split(np.random.choice(DEBUG_ROSLAUNCH_COMMANDS)))
    process = subprocess.check_call(command)
    return process


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-d', '--debug', action='store_true',
                            help="Run this script in debug mode")
    arg_parser.add_argument('--randomize_locations', action='store_true',
                            help="Randomize the locations of the robot")
    arg_parser.add_argument('--randomize_objects', action='store_true',
                            help="Randomize the locations of the objects")
    arg_parser.add_argument('--inject_movement', action='store_true',
                            help="Randomize the movement faults")
    arg_parser.add_argument('--inject_perception', action='store_true',
                            help="Randomize the perception faults")
    arg_parser.add_argument('--inject_beliefs', action='store_true',
                            help="Randomize the belief faults")
    arg_parser.add_argument('--no_fault_free', action='store_true',
                            help="Flag indicating that at least one fault must be injected")
    arg_parser.add_argument('--single_fault', action='store_true',
                            help="Only a single fault is injected")
    args = arg_parser.parse_args()
    injection_options = {
        "randomize_locations": args.randomize_locations,
        "randomize_objects": args.randomize_objects,
        "inject_movement": args.inject_movement,
        "inject_beliefs": args.inject_beliefs,
        "inject_perception": args.inject_perception,
    }

    assert not args.no_fault_free or args.inject_movement or args.inject_perception or args.inject_beliefs, \
        "no-fault-free cannot be true at the same time as no injected faults"

    main(injection_options, not args.no_fault_free, args.single_fault, args.debug)
