#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_step import AbstractStep

from .arm import ArmAction
from .background_task import BackgroundTaskAction
from .beep import BeepAction
from .find_closest_person import FindClosestPersonAction
from .find_grasps import FindGraspsAction
from .find_object import FindObjectAction
from .gripper import GripperAction
from .joystick_trigger import JoystickTriggerAction
from .listen import ListenAction
from .look import LookAction
from .look_at_closest_person import LookAtClosestPersonAction
from .look_at_gripper import LookAtGripperAction
from .move import MoveAction
from .pick import PickAction
from .place import PlaceAction
from .speak import SpeakAction
from .toggle_breakers import ToggleBreakersAction
from .torso import TorsoAction
from .wait import WaitAction


class Actions(object):
    """Registry of actions"""

    def __init__(self, registry):
        """
        Args:
            registry (dict) : This is a dict of name -> Action class mappings
        """
        self.registry = { key: klass() for key, klass in registry.iteritems() }

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


# The default actions contain all the action interfaces that are known to this
# package
default_actions_dict = {
    'arm': ArmAction,
    'background_task': BackgroundTaskAction,
    'beep': BeepAction,
    'find_closest_person': FindClosestPersonAction,
    'find_grasps': FindGraspsAction,
    'find_object': FindObjectAction,
    'gripper': GripperAction,
    'joystick_trigger': JoystickTriggerAction,
    'listen': ListenAction,
    'look': LookAction,
    'look_at_closest_person': LookAtClosestPersonAction,
    'look_at_gripper': LookAtGripperAction,
    'move': MoveAction,
    'pick': PickAction,
    'place': PlaceAction,
    'speak': SpeakAction,
    'toggle_breakers': ToggleBreakersAction,
    'torso': TorsoAction,
}

def get_default_actions():
    return Actions(default_actions_dict)
