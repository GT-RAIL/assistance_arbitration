#!/usr/bin/env python
# A constants file for various keys that could be in context dictionaries

from __future__ import print_function, division


# The trusty old objdict

class objdict(dict):
    """Provides class-like access to a dictionary"""
    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        if name in self:
            del self[name]
        else:
            raise AttributeError("No such attribute: " + name)


# The different sets of constants that are helpful for working with the messages
# defined in this package

TASK_CONTEXT_KEYS = objdict({
    # The key for a context dictionary within the context dictionary
    'CONTEXT': 'context',

    # The key for a resume hint value (which is one of
    # RequestAssistanceResult.RESUME_*)
    'RESUME_HINT': 'resume_hint',

    # The key for the name of the task in the context dictionary
    'TASK': 'task',

    # The key for the name of an action in the context dictionary
    'ACTION': 'action',

    # The key for the number of times a component may have aborted
    'NUM_ABORTS': 'num_aborts',

    # The key indicating the index of the step a task is/was executing
    'STEP_IDX': 'step_idx',

    # A key for the params sent to the task during execution
    'PARAMS': 'params',

    # A key for an exception, if any, that might've been encountered during
    # task execution
    'EXCEPTION': 'exception',

    # A key for the diagnosis details in the context dictionary
    'DIAGNOSIS': 'diagnosis',
})
