# Assistance Arbitration

Feeling alone? Need help? Don't worry, we've got you covered. Our novel algorithms will help you choose what's the best help for you! Step right up.

High-level structure:

- [`assistance_msgs`](assistance_msgs/) - The primary interface between all the other modules in this folder.
- [`task_executor`](task_executor/) - The executive layer of the robot for running fetch and deliver tasks according to a program specification in YAML (now; perhaps CodeIt! in the future).
- [`assistance_arbitrator`](assistance_arbitrator/) - Research code that uses context passed on from the task_executor (or another node that might be requesting assistance), and decides where to forward the request based on available request handling strategies. The result of handling the request is then forwarded back to the executive level to resume execution.
- `*_strategy` - The different strategies available for handling the requests for assistance followed by the manner of resuming execution.
- [`helpers`](helpers/) - Helper packages to this system. E.g.: packages to enforce parity between the real robot and simulation
- [`scripts`](scripts/) - Scripts and utilities to install this set of packages into the workspace (also install the workspace itself).
- [`docs/`](docs/) - reStructuredText documentation of the API; built with [Sphinx](http://www.sphinx-doc.org/en/master/)


## Setup

Prerequisites:

- Ubuntu 18.04
- ROS Melodic
- No `conda`
- SSH keys registered with Github

Then:

1. Navigate to the folder where you want to setup this workspace. The workspace technically consists of two chained workspaces - `active` and `stable`. This repository will be installed in the `active` workspace.
1. Copy the files `scripts/setup_ws.sh` and `scripts/requirements.txt` into this directory
1. Run `setup_ws.sh` from that directory


## Task Execution

The repository as a whole is a very involved executive level of a robot architecture. This section provides a very high level overview of that architecture.

![Package Structure](docs/package_structure.png)

1. Tasks, defined as sequences of robot actions, are assigned to the `task_executor` node using an [`ExecuteActionGoal`](assistance_msgs/action/Execute.action). The available tasks are generally defined at [`tasks.yaml`](task_executor/config/tasks.yaml)
1. The `task_executor` calls various actions, services, etc. in the robot system to accomplish the task. As part of the execution, there are 2 sources of information that the executor can use to parameterize its actions or the control flow:
    * The `database` node contains known locations and/or joint poses in the environment that can be used as inputs for actions. The entries in the `database` are generally available in [`data.yaml`](task_executor/config/data.yaml)
    * The `beliefs` node is designed to keep track of semantic aspects of the robot or world state through the progress of the task. Beliefs are continuous values between 0 and 1 that can be updated by any node in a non-Bayesian manner by publishing to `/execution_monitor/trace`. The beliefs that are tracked are defined at [`BeliefKeys.msg`](assistance_msgs/msg/BeliefKeys.msg)
1. If the task succeeds, or is preempted, the `task_executor` returns the corresponding status (`actionlib_msgs/GoalStatus`) and result
1. If the `task_executor` encounters an error, it generates a [`RequestAssistanceActionGoal`](assistance_msgs/action/RequestAssistance.action) for the `assisance_arbitrator` and then awaits a [`RequestAssistanceActionResult`](assistance_msgs/action/RequestAssistance.action) from it, which contains a `resume_hint` on how to (or not to) proceed with the task
1. When addressing a `RequestAssistanceActionGoal`, the `assisance_arbitrator` has the option of executing recovery actions itself. It can do this through the `recovery_executor`, which is simply another instantiation of the `task_executor`
1. In order to decide the recovery strategies to use, the `assisance_arbitrator` has access to a trace of tasks that have been completed or the faults that have manifested through the `execution_monitor`
    * The `execution_monitor` logs [`ExecutionEvent`](assistance_msgs/msg/ExecutionEvent.msg) messages, which are sent out on the topic `/execution_monitor/trace`
    * The `task_executor` publishes the status of each of its actions, as they are completed or not, to `/execution_monitor/trace`
    * Dedicated fault monitors also publish the fault statuses of different components to `/execution_monitor/trace`
    * Since belief updates are also sent out the `/execution_monitor/trace`, the `execution_monitor` contains a log of how the beliefs might change over the course of the task


## API Documentation

**WARNING: API docs are out of date and need to be rebuilt**

The API documentation of the packages in this folder is likely to go out of date very quickly as new functionality is developed. Therefore, we use `sphinx` to automatically generate documentation from docstrings in the source code.

**If you are developing new functionality in `task_executor` or `task_monitor`, please copy the precedent set by existing code in writing docstrings so that the API documentation can remain parse-able**

### Viewing the documentation

1. Make sure you have the necessary `pip` dependencies of `sphinx` installed. They are specified in [`setup_ws.sh`](../scripts/setup_ws.sh)
1. Navigate to [`docs`](docs/) and build the documentation: `make html`
1. Navigate to [`docs/build/html`](docs/build/html) (this folder will be generated by the previous command) and start a simple server: `python -m SimpleHTTPServer`.
1. Navigate to [`http://localhost:8000`](http://localhost:8000) in your browser.

### Adding to the documentation

Since the docs are generated with [`sphinx`](https://www.sphinx-doc.org/en/master/index.html), it's a good idea to get familiar with it, and with reStructuredText, which is format used to specify each page of the documentation (think of the latter as markdown++). Here are some resources:

- [Intro to reStructuredText](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html)
- [Common "commands" for reStructuredText](https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html)
- [Python specific "commands"](https://www.sphinx-doc.org/en/master/usage/restructuredtext/domains.html#the-python-domain)
- [reStructuredText cheat sheet](https://github.com/ralsina/rst-cheatsheet/blob/master/rst-cheatsheet.rst)

The most frequent update that is likely to happen to the documentation is with the addition or changes of actions in tasks, so here are the steps that one should follow for that:

1. Create your action file in [`task_executor.actions`](task_executor/src/task_executor/actions/)
1. Define your action `class`, which should derive from [`AbstractStep`](task_executor/src/task_executor/abstract_step.py). Make sure to follow the precedent set by the actions that are already in the code base
1. Add the action's name and `class` to [`__init__.py`](task_executor/src/task_executor/actions/__init__.py)
1. Add the action to the actions documentation page [`task_executor.actions.rst`](docs/source/task_executor.actions.rst). Follow the syntax of other actions that are already in that file.


## TODO

Some of the things that would be good to figure out, but I don't know how to achieve, or do not have the bandwidth for:

- Figure out a [`dynamic_reconfigure`](http://wiki.ros.org/dynamic_reconfigure)-like API to the reload functionality provided by a lot of the servers. Unfortunately, `dynamic_reconfigure` itself does not allow for the dynamic reconfiguration of dictionaries or lists at the moment.
- Make the monitoring and executive levels truly general; among the biggest problems are:
    - Beliefs: automatically including beliefs in the messages, and updating the appropriate belief as part of the task
    - Database: including and parsing custom data types from the database dictionary
    - Actions: including primitive actions into the global module search path for the actions. We have to have the same actions available via direct access as are available through the task execution server
    - Ops: ops need to be more than just predefined functions within the package
    - Background monitors: Background monitors that are tracked by the execution monitor should be an arbitrary list of monitors
- Include a resource aware manager, like [Playful](https://github.com/vincentberenz/playful), as a concurrent execution manager
