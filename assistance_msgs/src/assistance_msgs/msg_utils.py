#!/usr/bin/env python
# Python utilities to help in working with the messages in this package

from __future__ import print_function, division

from assistance_msgs.msg import RequestAssistanceResult


# The different utility functions

def pprint_context(context, return_types=True):
    """
    Helper function to pretty print the variable context that is returned
    from the tasks. Basically stub out all objects that are not basic python
    types. Also convert all unicode strings to str (to assist in the CLI)

    Args:
        context (dict, list, tuple) : A container of context that form
        the context of return values from a task or action
        return_types (bool) : Return the type of a non-basic python types; else
        ignore the key containing the non-basic type

    Returns:
        (dict, list, tuple) : A container of context with all \
            values that are not basic python types stubbed out
    """
    if isinstance(context, dict):
        pp_context = {}
        for k, v in context.iteritems():
            if isinstance(v, (list, tuple, dict,)):
                pp_context[k] = pprint_context(v, return_types)
            elif isinstance(v, (bool, int, long, float, str, unicode)):
                if not isinstance(v, unicode):
                    pp_context[k] = v
                else:
                    pp_context[k] = str(v)
            elif return_types:
                pp_context[k] = type(v)

    elif isinstance(context, (list, tuple,)):
        pp_context = []
        for x in context:
            if isinstance(x, (list, tuple, dict,)):
                pp_context.append(pprint_context(x, return_types))
            elif isinstance(x, (bool, int, long, float, str, unicode)):
                if not isinstance(x, unicode):
                    pp_context.append(x)
                else:
                    pp_context.append(str(x))
            elif return_types:
                pp_context.append(type(x))

    return pp_context


def get_final_component_context(goal_context):
    """
    Given a :attr:`RequestAssistanceGoal.context` as input, return the context
    of the final component in the context chain

    Args:
        goal_context (dict)

    Returns
        (dict)
    """
    if len(goal_context.get('context', {})) > 0:
        return get_final_component_context(goal_context['context'])
    else:
        return goal_context


def get_number_of_component_aborts(goal_context):
    """
    Given the hierarchy of tasks in the goal context, obtain a vector of the
    number of failures in each part of the component of the hierarchy. The
    first index maps to the highest level of the hierarcy and the last
    index maps to the lowest level of the hierarchy.

    Args:
        goal_context (dict) : the goal context

    Returns:
        (tuple):
            - component_names (list) a list of component names from highest \
                in the task hierarchy to the lowest
            - num_aborts (list) a list of the number of times each \
                component in component_names aborted
    """
    component_names = []
    num_aborts = []
    sub_context = goal_context

    while sub_context is not None and isinstance(sub_context, dict):
        component_names.append(sub_context.get('task') or sub_context.get('action'))
        num_aborts.append(sub_context.get('num_aborts'))
        sub_context = sub_context.get('context')

    # Return the lists
    return (component_names, num_aborts,)


def create_continue_result_context(goal_context):
    """
    Given the context of a ``assistance_msgs/RequestAssistanceGoal``
    return a dictionary for a ``assistance_msgs/RequestAssistanceResult``
    context that indicates :const:`RequestAssistanceResult.RESUME_CONTINUE`

    Args:
        goal_context (dict) : the goal context

    Return:
        (dict) : the result context
    """
    if 'task' in goal_context:
        return {
            'task': goal_context['task'],
            'step_idx': goal_context['step_idx'],
            'resume_hint': RequestAssistanceResult.RESUME_CONTINUE,
            'context': create_continue_result_context(goal_context['context']),
        }
    else:
        return {}

def set_task_hint_in_context(result_context, task_name, resume_hint):
    """
    Given a result context dictionary, mark the desired task name with the
    desired resume hint

    Args:
        result_context (dict) : the result context, possibly created by
            :meth:`create_continue_result_context`
        task_name (str) : the name of the task
        resume_hint (uint8) : A ``assistance_msgs/RequestAssistanceResult`` \
            resume_hint constant for the task's resume hint

    Returns:
        (dict) : a result context dictionary with the task set to the \
            desired resume_hint. Note: we do not copy, so the incoming arg \
            might also get affected

    Raises:
        KeyError : if :data:`task_name` is not found in the context
    """

    # Error checking
    if 'task' not in result_context:
        raise KeyError("Expected a result context for tasks. Not found in {}!".format(result_context))

    # If this is not the task we want, then continue on to its context.
    # Otherwise, mark this task as the one we want to update and return
    if result_context['task'] == task_name:
        result_context['resume_hint'] = resume_hint
    else:
        result_context['context'] = set_task_hint_in_context(result_context['context'], task_name, resume_hint)

    return result_context
