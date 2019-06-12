#!/usr/bin/env python
# Decides which remote controller to use

from __future__ import print_function, division

import os
import sys
import pkgutil
import importlib

import rospy

from .controller import RemoteController as _DefaultController


# Automatically fetch the remote controller that we should use. If we cannot
# fetch the desired external controller, then use the default one in this
# package
def _fetch_remote_controller():
    try:
        pkgname = rospy.get_param('/remote_strategy/remote_controller')
        pkg = importlib.import_module(pkgname)
        controller_class = getattr(pkg, "RemoteController")
    except Exception as e:
        print("Remote: Unable to fetch controller - {}".format(e), file=sys.stderr)
        controller_class = _DefaultController

    return controller_class

RemoteController = _fetch_remote_controller()
