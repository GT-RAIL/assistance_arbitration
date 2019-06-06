#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function, division

import os
import sys
import pkgutil
import importlib

import rospy

from task_executor.abstract_step import AbstractStep


# Automatically fetch actions in the param to populate the default_actions_dict
# If we cannot fetch the actions from an external package, then we load the
# default actions in this package, which *should* be considerably fewer
pkgname = __package__
pkg = sys.modules[__name__]
pkgpath = os.path.dirname(__file__)
action_names = action_class_names = []
default_actions_dict = {}


def _populate_default_actions():
    """Populate the default actions if they were not already defined"""
    global action_names, action_class_names, default_actions_dict

    # First find all the packages in the directory
    found_packages = sorted(
        [(i, x) for i, x, _ in pkgutil.iter_modules([pkgpath], "{}.".format(pkgname))],
        key=lambda t: t[-1]
    )
    action_names = [x.split('.')[-1] for _, x in found_packages]
    importer = found_packages[0][0]

    # Then infer the standard class names of the actions
    action_class_names = [
        "{}Action".format("".join([x.capitalize() for x in name.strip().split('_')]))
        for name in action_names
    ]

    # And after that, we populate the valid ones in the dictionary
    for idx, name in enumerate(action_names):
        action = None
        try:
            module = importer.find_module(found_packages[idx][1]).load_module(found_packages[idx][1])
            action = getattr(module, action_class_names[idx])
        except (ImportError, AttributeError) as e:
            print("Actions: Exception({}) importing {} - {}".format(type(e).__name__, name, e), file=sys.stderr)
        else:
            default_actions_dict[name] = action


try:
    # Fetch the name of the target package from the global ROS param
    pkgname = rospy.get_param('~actions', None) or rospy.get_param('/task_executor/actions')
    pkg = importlib.import_module(pkgname)
    pkgpath = os.path.dirname(pkg.__file__)

    # Check to see if the hard work has already been done for us
    default_actions_dict = getattr(pkg, "default_actions_dict", {})
    action_names = getattr(pkg, "action_names", [])
    action_class_names = getattr(pkg, "action_class_names", [])

    # If we have to do the hard work, then make sure that the target package
    # is indeed a directory
    if len(default_actions_dict) == 0 or len(action_names) == 0 or len(action_class_names) == 0:
        assert os.path.isdir(pkgpath), "{} is not a directory! Cannot parse actions".format(pkgpath)
except Exception as e:
    print("Actions: Exception({}) fetching external actions - {}".format(type(e).__name__, e), file=sys.stderr)
finally:
    if len(default_actions_dict) == 0 or len(action_names) == 0 or len(action_class_names) == 0:
        _populate_default_actions()

print("Actions: {}/{} actions imported".format(len(default_actions_dict), len(action_names)))


# Finally, we can initialize the container for all the actions

class Actions(object):
    """
    A registry of actions. It is recommended that you create this object with
    :func:`get_default_actions`. In order to use the actions, they must be
    intialized, which includes connecting to their action servers, services,
    etc. You can use the :attr:`initialized` attribute of this object to know
    if the actions are initialized or not.
    """

    def __init__(self, registry):
        """
        Args:
            registry (dict) : This is a dict of name -> Action class mappings
        """
        self.registry = { key: klass() for key, klass in registry.iteritems() }

        # Flag for if the actions are initialized
        self.initialized = False

        # Quick sanity check because I don't trust people. Also set the action
        # as an attribute for '.' based referencing
        for key, action in self.registry.iteritems():
            assert isinstance(action, AbstractStep)
            setattr(self, key, action)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self):
        for key, action in self.registry.iteritems():
            action.init(key)

        # Mark the actions as initialized
        self.initialized = True


def get_default_actions():
    """
    Provides a consistent manner of fetching all the actions that are available
    on the robot. This is the preferred manner of getting the actions:

    >>> actions = get_default_actions()
    >>> move_params = { 'location': 'waypoints.origin' }
    >>> actions.move(**move_params)

    Returns:
        (Actions) : The default registry of all the actions that are available
    """
    return Actions(default_actions_dict)
