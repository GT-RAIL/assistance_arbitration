#!/usr/bin/env python
# Decides which recovery strategies to use

from __future__ import print_function, division

import os
import sys
import pkgutil
import importlib

import rospy

from .recovery_strategies import RecoveryStrategies as _DefaultStrategies


# Automatically fetch the recovery strategies that we should use. If we cannot
# fetch the desired external strategies, then use the default ones in this
# package
def _fetch_recovery_strategies():
    try:
        pkgname = rospy.get_param('/predefined_strategy/recovery_strategies')
        pkg = importlib.import_module(pkgname)
        strategies_class = getattr(pkg, "RecoveryStrategies")
    except Exception as e:
        print("Predefined: Unable to fetch recoveries - {}".format(e), file=sys.stderr)
        strategies_class = _DefaultStrategies

    return strategies_class

RecoveryStrategies = _fetch_recovery_strategies()
