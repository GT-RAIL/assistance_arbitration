#!/usr/bin/env python
# Provide an interface to the robot for remote assistance

from __future__ import print_function, division

import os
import sys
import time
import json
import pickle
import signal

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from assistance_msgs.msg import (RequestAssistanceResult, InterventionEvent,
                                 InterventionHypothesisMetadata,
                                 InterventionActionMetadata, BeliefKeys)
from assistance_msgs.srv import (EnableRemoteControl,
                                 EnableRemoteControlResponse,
                                 DisableRemoteControl,
                                 DisableRemoteControlResponse)
from std_srvs.srv import Trigger, TriggerResponse

from task_executor.actions import get_default_actions

# Plotly and Dash
import plotly.graph_objs as go
import dash
import dash_core_components as dcc
import dash_html_components as html

# Import isolation
sys.path.append('/home/banerjs/Libraries/RAIL/codebase/banerjs/isolation/models')
from isolation.data.annotations import Annotations


# The app class that contains the app configuration and the controller

APP = dash.Dash(
    __name__,
    external_stylesheets=[
        {
            'href': 'https://stackpath.bootstrapcdn.com/bootstrap/4.1.3/css/bootstrap.min.css',
            'rel': 'stylesheet',
            'integrity': 'sha384-MCw98/SFnGE8fJT3GXwEOngsV7Zt27NXFoaoApmYm81iuXoPkFOJwJ8ERdknLPMO',
            'crossorigin': 'anonymous',
        },
    ]
)


class RemoteController(object):
    """
    Create a flask server object and the ROS node to interface with the robot
    during a remote debugging episode.
    """

    # Flask
    APP_HOST = '0.0.0.0'
    APP_PORT = 8080

    # Specifying hypotheses
    MAX_NUM_HYPOTHESES = 6

    # The trace topic
    INTERVENTION_TRACE_TOPIC = '/intervention_monitor/trace'

    # The services to enable and disable this controller
    ENABLE_SERVICE = '/remote_controller/enable'
    DISABLE_SERVICE = '/remote_controller/disable'

    # The service to call when the remote controller needs to indicate that the
    # recovery is complete and that it should be polled for the resume strategy.
    # Ideally, this would be specified somewhere other than here
    INTERVENTION_COMPLETE_SERVICE = '/remote_strategy/intervention_complete'

    # RViz topics
    RELOCALIZATION_TOPIC = '/initialpose'
    MOVE_GOAL_TOPIC = '/move_base_simple/goal'

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP

        # Create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

        # The publisher of the trace
        self._trace_pub = rospy.Publisher(
            RemoteController.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            queue_size=10
        )

        # Service proxy to indicate that the intervention is complete
        self._complete_intervention_srv = rospy.ServiceProxy(RemoteController.INTERVENTION_COMPLETE_SERVICE, Trigger)

        # Flags and services to enable and disable this controller
        self._current_error = None
        self._current_response = None
        self._enable_service = rospy.Service(RemoteController.ENABLE_SERVICE, EnableRemoteControl, self.enable)
        self._disable_service = rospy.Service(RemoteController.DISABLE_SERVICE, DisableRemoteControl, self.disable)

        # The robot controller
        self.controller = RobotController(get_default_actions(),
                                          intervention_trace_pub=self._trace_pub)
        self._action_buttons = None      # Buttons for controller actions
        self._completion_buttons = None  # Buttons for ending the intervention

        # Register a subscriber to the localization and goal interfaces on RViz
        self._relocalize_subscriber = rospy.Subscriber(
            RemoteController.RELOCALIZATION_TOPIC,
            PoseWithCovarianceStamped,
            self._on_relocalize
        )
        self._move_goal_subscriber = rospy.Subscriber(
            RemoteController.MOVE_GOAL_TOPIC,
            PoseStamped,
            self._on_move_goal
        )

        # Initialize the application
        self._define_app()

    def start(self):
        self.controller.start()
        self._app.run_server(host=RemoteController.APP_HOST,
                             port=RemoteController.APP_PORT,
                             debug=False)

    def stop(self, *args, **kwargs):
        self.controller.stop()

        # Give some time for rospy to shutdown
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def enable(self, req=None):
        self._current_error = req.request
        if self._current_error.context:
            self._current_error.context = pickle.loads(self._current_error.context)
        self._current_response = None
        self.controller.enable()
        return EnableRemoteControlResponse()

    def disable(self, req=None):
        self._current_error = None
        # This happens if the RViz window is closed and there is no recovery
        # strategy that is provided. Default is to then exit from the task
        if self._current_response is None:
            self._current_response = RequestAssistanceResult(resume_hint=RequestAssistanceResult.RESUME_NONE)
        self.controller.disable()
        return DisableRemoteControlResponse(response=self._current_response)

    def _define_app(self):
        """
        Define the app to control the robot.
        """

        # First the section for specifying the hypotheses
        hypothesis_layout = html.Div(
            [html.H3('Failure Information', className='row'),
             dcc.Markdown('', id='failure-information')] +
            [html.H3('Fault Hypotheses', className='row')] +
            [html.Div([
                html.Div(
                    [html.Div([
                        dcc.Dropdown(id='hypothesis_{}_select'.format(idx),
                                     options=Annotations.RESULT_OPTIONS,
                                     value=None,
                                     className="dropdown form-control-sm"),
                        dcc.Input(id='hypothesis_{}_value'.format(idx), style={'display': 'none'}),
                     ], id='hypothesis_{}'.format(idx), className='my-4')
                     for idx in xrange(RemoteController.MAX_NUM_HYPOTHESES)],
                    className='col-8'
                ),
                dcc.Checklist(
                    options=[
                        { 'label': 'Confirmed', 'value': idx }
                        for idx in xrange(RemoteController.MAX_NUM_HYPOTHESES)
                    ],
                    values=[],
                    id='hypotheses_certain_select',
                    className='my-3 col-4',
                    labelClassName='my-3'
                ),
                dcc.Checklist(
                    id="hypotheses_certain_value",
                    style={'display': 'none'},
                    values=[],
                    options=[{'label': idx, 'value': idx} for idx in xrange(RemoteController.MAX_NUM_HYPOTHESES)]
                ),
            ], id='hypotheses', className='row'
            )] +
            [dcc.Interval(id='interval-component', n_intervals=0, interval=10),
             dcc.Input(value="", id="enable-component", style={'display': 'none'})],
             style={
                'float': 'left',
                'width': '30%',
                'position': 'fixed',
             },
             className='container'
        )

        # Then the section(s) for taking actions on the robot
        actions_layout = html.Div([
            html.Div([
                html.H4("Look", className='text-center'),
                html.Div([
                    html.Button("Left", id='look-left-action', className='offset-1 col-1 btn btn-default'),
                    html.Button("Right", id='look-right-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Up", id='look-up-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Down", id='look-down-action', className='offset-2 col-1 btn btn-default'),
                ], className='row'),
            ], className='container'),

            html.Div([
                html.H4("Move", className='text-center mt-5'),
                html.Div([
                    html.Button("Left", id='move-left-action', className='offset-1 col-1 btn btn-default'),
                    html.Button("Right", id='move-right-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Forward", id='move-forward-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Backward", id='move-backward-action', className='offset-2 col-1 btn btn-default'),
                ], className='row'),
            ], className='container'),
            # Move with waypoint will be set based on the RViz input
            # Relocalize will be set based on RViz input

            html.Div([
                html.H4("Torso", className='text-center mt-5'),
                html.Div([
                    html.Button("Up", id='torso-up-action', className='offset-4 col-1 btn btn-default'),
                    html.Button("Down", id='torso-down-action', className='offset-2 col-1 btn btn-default'),
                ], className='row'),
            ], className='container'),

            html.Div([
                html.H4("Arm Movement", className='text-center mt-5'),
                html.Div([
                    html.Button("Tuck", id='arm-position-tuck-action', className='offset-4 col-1 btn btn-default'),
                    html.Button("Ready", id='arm-position-ready-action', className='offset-2 col-1 btn btn-default'),
                ], className='row'),
                html.Div([
                    html.Button("Forward", id='arm-linear-forward-action', className='offset-1 col-1 btn btn-default'),
                    html.Button("Backward", id='arm-linear-backward-action', className='col-1 btn btn-default'),
                    html.Button("Up", id='arm-linear-up-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Down", id='arm-linear-down-action', className='col-1 btn btn-default'),
                    html.Button("Left", id='arm-linear-left-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Right", id='arm-linear-right-action', className='col-1 btn btn-default'),
                ], className='row'),
                html.Div([
                    html.Button("Roll Left", id='arm-angular-roll-left-action', className='offset-1 col-1 btn btn-default'),
                    html.Button("Roll Right", id='arm-angular-roll-right-action', className='col-1 btn btn-default'),
                    html.Button("Pitch Down", id='arm-angular-pitch-down-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Pitch Up", id='arm-angular-pitch-up-action', className='col-1 btn btn-default'),
                    html.Button("Yaw Left", id='arm-angular-yaw-left-action', className='offset-2 col-1 btn btn-default'),
                    html.Button("Yaw Right", id='arm-angular-yaw-right-action', className='col-1 btn btn-default'),
                ], className='row'),
            ], className='container'),

            html.Div([
                html.H4("Perception", className='text-center mt-5'),
                html.Div([
                    html.Button("Crop", id='crop-action', className='offset-4 col-1 btn btn-default'),
                    html.Button("Segment", id='segment-action', className='offset-2 col-1 btn btn-default'),
                ], className='row'),
            ], className='container'),

            html.Div([
                html.H4("Beliefs", className='text-center mt-5'),
                html.Div([
                    html.Button("Cube@Pickup", id='belief-cube-pickup-action', className='offset-1 col-2 btn btn-default'),
                    html.Button("~Cube@Pickup", id='belief-cube-not-pickup-action', className='col-2 btn btn-default'),
                    html.Button("Cube@Dropoff", id='belief-cube-dropoff-action', className='offset-2 col-2 btn btn-default'),
                    html.Button("~Cube@Dropoff", id='belief-cube-not-dropoff-action', className='col-2 btn btn-default'),
                ], className='row'),
                html.Div([
                    html.Button("Door Open", id='belief-door-open-action', className='offset-4 col-2 btn btn-default'),
                    html.Button("Door Closed", id='belief-door-closed-action', className='col-2 btn btn-default'),
                ], className='row'),
            ], className='container'),

            html.Div([
                html.H4(["Task Actions ", html.Span("(ends intervention)", className='small')],
                        className='text-center mt-5'),
                html.Div([
                    html.Button("Retry", id='retry-action', className='offset-1 col-2 btn btn-default'),
                    html.Button("Restart", id='restart-action', className='offset-2 col-2 btn btn-default'),
                    html.Button("Abort", id='abort-action', className='offset-2 col-2 btn btn-default'),
                ], className='row'),
            ], className='container'),

        ],
        style={
            'marginLeft': '31%',
        })

        # Log all the buttons that we have defined above and associate them with
        # actions available in the controller
        self._action_buttons = {
            'look-left-action': self.controller.look_left,
            'look-right-action': self.controller.look_right,
            'look-up-action': self.controller.look_up,
            'look-down-action': self.controller.look_down,

            'move-left-action': self.controller.move_left,
            'move-right-action': self.controller.move_right,
            'move-forward-action': self.controller.move_forward,
            'move-backward-action': self.controller.move_backward,

            'torso-up-action': self.controller.torso_up,
            'torso-down-action': self.controller.torso_down,

            'arm-position-tuck-action': self.controller.arm_position_tuck,
            'arm-position-ready-action': self.controller.arm_position_ready,
            'arm-linear-forward-action': self.controller.arm_linear_forward,
            'arm-linear-backward-action': self.controller.arm_linear_backward,
            'arm-linear-up-action': self.controller.arm_linear_up,
            'arm-linear-down-action': self.controller.arm_linear_down,
            'arm-linear-left-action': self.controller.arm_linear_left,
            'arm-linear-right-action': self.controller.arm_linear_right,
            'arm-angular-roll-left-action': self.controller.arm_angular_roll_left,
            'arm-angular-roll-right-action': self.controller.arm_angular_roll_right,
            'arm-angular-pitch-down-action': self.controller.arm_angular_pitch_down,
            'arm-angular-pitch-up-action': self.controller.arm_angular_pitch_up,
            'arm-angular-yaw-left-action': self.controller.arm_angular_yaw_left,
            'arm-angular-yaw-right-action': self.controller.arm_angular_yaw_right,

            'crop-action': self.controller.crop_image,
            'segment-action': self.controller.segment_image,

            'belief-cube-pickup-action': self.controller.noop,
            'belief-cube-not-pickup-action': self.controller.noop,
            'belief-cube-dropoff-action': self.controller.noop,
            'belief-cube-not-dropoff-action': self.controller.noop,
            'belief-door-open-action': self.controller.noop,
            'belief-door-closed-action': self.controller.noop,
        }

        # Log the completion buttons that we have defined above and associate
        # them with resumption hints
        self._completion_buttons = {
            'retry-action': RequestAssistanceResult.RESUME_CONTINUE,
            'restart-action': RequestAssistanceResult.RESUME_RETRY,
            'abort-action': RequestAssistanceResult.RESUME_NONE,
        }

        # The final layout of the interface
        self._app.layout = html.Div([hypothesis_layout, actions_layout])

        # Then register callbacks for each of the buttons
        self._app.callback(
            dash.dependencies.Output('failure-information', 'children'),
            [dash.dependencies.Input('enable-component', 'value')]
        )(self._define_failure_information_callback())

        self._app.callback(
            dash.dependencies.Output('enable-component', 'value'),
            [dash.dependencies.Input('interval-component', 'n_intervals')]
        )(self._define_enable_component_callback())

        self._app.callback(
            dash.dependencies.Output('hypotheses', 'style'),
            [dash.dependencies.Input('enable-component', 'value')]
        )(self._define_hypothesis_enable_callback())

        for button_id, button_cb in self._action_buttons.iteritems():
            self._app.callback(
                dash.dependencies.Output(button_id, 'disabled'),
                [dash.dependencies.Input('enable-component', 'value')]
            )(self._define_button_enable_callback())

            self._app.callback(
                dash.dependencies.Output(button_id, 'autoFocus'),
                [dash.dependencies.Input(button_id, 'n_clicks')]
            )(self._define_action_button_callback(button_id, button_cb))

        for button_id, resume_hint in self._completion_buttons.iteritems():
            self._app.callback(
                dash.dependencies.Output(button_id, 'disabled'),
                [dash.dependencies.Input('enable-component', 'value')]
            )(self._define_button_enable_callback())

            self._app.callback(
                dash.dependencies.Output(button_id, 'autoFocus'),
                [dash.dependencies.Input(button_id, 'n_clicks')]
            )(self._define_completion_button_callback(button_id, resume_hint))

        for idx in xrange(RemoteController.MAX_NUM_HYPOTHESES):
            name = 'hypothesis_{}'.format(idx)

            self._app.callback(
                dash.dependencies.Output('{}_value'.format(name), 'value'),
                [dash.dependencies.Input('{}_select'.format(name), 'value')],
                [dash.dependencies.State('{}_value'.format(name), 'value')]
            )(self._define_hypothesis_selected_callback())

        self._app.callback(
            dash.dependencies.Output('hypotheses_certain_value', 'values'),
            [dash.dependencies.Input('hypotheses_certain_select', 'values')],
            [dash.dependencies.State('hypotheses_certain_value', 'values')] +
            [dash.dependencies.State('hypothesis_{}_value'.format(idx), 'value')
             for idx in xrange(RemoteController.MAX_NUM_HYPOTHESES)]
        )(self._define_hypothesis_certain_callback())

    def _define_enable_component_callback(self):
        def enable_component(*args):
            return "Enabled" if self._current_error is not None else "Disabled"
        return enable_component

    def _define_failure_information_callback(self):
        def failure_information(enabled):
            # If this is disabled, return nothing
            if enabled != "Enabled":
                return ""

            # Just print out the keys from the assistance message
            message = """
**Component**: {x.component}

**Status**: {x.component_status}

**Context**:
```
{json}
```
""".format(x=self._current_error, json=json.dumps(self._current_error.context or {},
                                                  indent=0,
                                                  separators=(',', ':')))
            return message

            # Otherwise, parse out the data from the assistance request
        return failure_information

    def _define_button_enable_callback(self):
        def button_enable(enabled):
            return (enabled != "Enabled")
        return button_enable

    def _define_hypothesis_enable_callback(self):
        def hypothesis_enable(enabled):
            return {} if enabled == 'Enabled' else {'display': 'none'}
        return hypothesis_enable

    def _define_action_button_callback(self, button_id, button_cb):
        def action_button(n_clicks):
            if self._current_error is not None:
                button_cb()
            return True
        return action_button

    def _define_completion_button_callback(self, button_id, resume_hint):
        def completion_button(n_clicks):
            if self._current_error is not None:
                assert self._current_response is None, "Current Response: {}".format(self._current_response)
                self._current_response = RequestAssistanceResult(resume_hint=resume_hint)
                self._complete_intervention_srv()
            return True
        return completion_button

    def _define_hypothesis_selected_callback(self):
        def hypothesis_selected(hypothesis, old_hypothesis):
            if self._current_error is not None:
                if hypothesis is not None:
                    self._send_hypothesis_event(
                        hypothesis,
                        InterventionHypothesisMetadata.SUSPECTED
                    )
                else:  # Hypothesis has been removed as a candidate
                    assert old_hypothesis is not None, "Both hypothesis and old_hypothesis are None"
                    self._send_hypothesis_event(
                        old_hypothesis,
                        InterventionHypothesisMetadata.ABSENT
                    )

            return hypothesis
        return hypothesis_selected

    def _define_hypothesis_certain_callback(self):
        def hypothesis_certain(certain_idx, old_certain_idx, *hypotheses):
            if self._current_error is not None:
                cidx_set = set(certain_idx)
                ocidx_set = set(old_certain_idx)

                # First mark those hypotheses that are no longer certain
                for idx in (ocidx_set - cidx_set):
                    if hypotheses[idx] is not None:
                        self._send_hypothesis_event(
                            hypotheses[idx],
                            InterventionHypothesisMetadata.SUSPECTED
                        )

                # Then make certain those hypotheses that were suspected
                for idx in (cidx_set - ocidx_set):
                    if hypotheses[idx] is not None:
                        self._send_hypothesis_event(
                            hypotheses[idx],
                            InterventionHypothesisMetadata.CONFIRMED
                        )

            return certain_idx
        return hypothesis_certain

    def _send_hypothesis_event(self, hypothesis, status):
        trace_msg = InterventionEvent(stamp=rospy.Time.now(),
                                      type=InterventionEvent.HYPOTHESIS_EVENT)
        trace_msg.hypothesis_metadata.name = hypothesis
        trace_msg.hypothesis_metadata.status = status
        self._trace_pub.publish(trace_msg)

    def _on_relocalize(self, msg):
        """Relocalization action taken on RViz"""
        if self._current_error is None:
            return

        trace_msg = InterventionEvent(stamp=msg.header.stamp,
                                      type=InterventionEvent.ACTION_EVENT)
        trace_msg.action_metadata.type = InterventionActionMetadata.RELOCALIZE
        trace_msg.action_metadata.args = pickle.dumps(msg)
        self._trace_pub.publish(trace_msg)

    def _on_move_goal(self, msg):
        """Move base goal provided on RViz"""
        if self._current_error is None:
            return

        trace_msg = InterventionEvent(stamp=msg.header.stamp,
                                      type=InterventionEvent.ACTION_EVENT)
        trace_msg.action_metadata.type = InterventionActionMetadata.MOVE_WAYPOINT
        trace_msg.action_metadata.args = pickle.dumps(msg)
        self._trace_pub.publish(trace_msg)


# A class to map the actions of the buttons to robot actions

class RobotController(object):
    """
    Provides semantically meaningful labels to the actions that are available
    to the user through `RemoteController`; also executes them.
    """

    LOOK_TILT_STEP = 0.1
    LOOK_PAN_STEP = 0.1

    TORSO_STEP = 0.1

    MOVE_LINEAR_STEP = 0.1
    MOVE_ANGULAR_STEP = 0.1

    ARM_LINEAR_STEP = 0.1
    ARM_ANGULAR_STEP = 0.1

    def __init__(self, actions, intervention_trace_pub=None):
        self._enabled = False

        # First get the actions that have been defined in the task_executor
        self.actions = actions

        # Setup trace publishers
        self._intervention_trace_pub = intervention_trace_pub

    def start(self):
        # self.actions.init()
        pass

    def stop(self):
        pass

    def enable(self):
        self._enabled = True

    def disable(self):
        self._disabled = False

    def _update_intervention_trace(self, action_type, **kwargs):
        # We only update the intervention trace because the execution trace is
        # automatically updated via the abstract step
        if self._intervention_trace_pub is None:
            return

        trace_msg = InterventionEvent(stamp=rospy.Time.now(),
                                      type=InterventionEvent.ACTION_EVENT)
        trace_msg.action_metadata.type = action_type
        trace_msg.action_metadata.args = pickle.dumps(kwargs)
        self._intervention_trace_pub.publish(trace_msg)

    def noop(self):
        pass

    def look_up(self):
        self.actions.look_pan_tilt(tilt_amount=-RobotController.LOOK_TILT_STEP)
        self._update_intervention_trace(InterventionActionMetadata.LOOK_UP)

    def look_down(self):
        self.actions.look_pan_tilt(tilt_amount=RobotController.LOOK_TILT_STEP)
        self._update_intervention_trace(InterventionActionMetadata.LOOK_DOWN)

    def look_left(self):
        self.actions.look_pan_tilt(pan_amount=RobotController.LOOK_PAN_STEP)
        self._update_intervention_trace(InterventionActionMetadata.LOOK_LEFT)

    def look_right(self):
        self.actions.look_pan_tilt(pan_amount=-RobotController.LOOK_PAN_STEP)
        self._update_intervention_trace(InterventionActionMetadata.LOOK_RIGHT)

    def move_forward(self):
        self.actions.move_planar(linear_amount=RobotController.MOVE_LINEAR_STEP)
        self._update_intervention_trace(InterventionActionMetadata.MOVE_FORWARD)

    def move_backward(self):
        self.actions.move_planar(linear_amount=-RobotController.MOVE_LINEAR_STEP)
        self._update_intervention_trace(InterventionActionMetadata.MOVE_BACKWARD)

    def move_left(self):
        self.actions.move_planar(angular_amount=RobotController.MOVE_ANGULAR_STEP)
        self._update_intervention_trace(InterventionActionMetadata.MOVE_LEFT)

    def move_right(self):
        self.actions.move_planar(angular_amount=-RobotController.MOVE_ANGULAR_STEP)
        self._update_intervention_trace(InterventionActionMetadata.MOVE_RIGHT)

    def torso_up(self):
        self.actions.torso_linear(amount=RobotController.TORSO_STEP)
        self._update_intervention_trace(InterventionActionMetadata.TORSO_UP)

    def torso_down(self):
        self.actions.torso_linear(amount=-RobotController.TORSO_STEP)
        self._update_intervention_trace(InterventionActionMetadata.TORSO_DOWN)

    def arm_linear_up(self):
        self.actions.arm_cartesian(linear_amount=[0, 0, RobotController.ARM_LINEAR_STEP])
        self._update_intervention_trace(InterventionActionMetadata.ARM_LINEAR_UP)

    def arm_linear_down(self):
        self.actions.arm_cartesian(linear_amount=[0, 0, -RobotController.ARM_LINEAR_STEP])
        self._update_intervention_trace(InterventionActionMetadata.ARM_LINEAR_DOWN)

    def arm_linear_left(self):
        self.actions.arm_cartesian(linear_amount=[0, -RobotController.ARM_LINEAR_STEP, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_LINEAR_LEFT)

    def arm_linear_right(self):
        self.actions.arm_cartesian(linear_amount=[0, RobotController.ARM_LINEAR_STEP, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_LINEAR_RIGHT)

    def arm_linear_forward(self):
        self.actions.arm_cartesian(linear_amount=[RobotController.ARM_LINEAR_STEP, 0, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_LINEAR_FORWARD)

    def arm_linear_backward(self):
        self.actions.arm_cartesian(linear_amount=[-RobotController.ARM_LINEAR_STEP, 0, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_LINEAR_BACKWARD)

    def arm_angular_roll_left(self):
        self.actions.arm_cartesian(angular_amount=[RobotController.ARM_ANGULAR_STEP, 0, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_ANGULAR_ROLL_LEFT)

    def arm_angular_roll_right(self):
        self.actions.arm_cartesian(angular_amount=[-RobotController.ARM_ANGULAR_STEP, 0, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_ANGULAR_ROLL_RIGHT)

    def arm_angular_pitch_down(self):
        self.actions.arm_cartesian(angular_amount=[0, RobotController.ARM_ANGULAR_STEP, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_ANGULAR_PITCH_DOWN)

    def arm_angular_pitch_up(self):
        self.actions.arm_cartesian(angular_amount=[0, -RobotController.ARM_ANGULAR_STEP, 0])
        self._update_intervention_trace(InterventionActionMetadata.ARM_ANGULAR_PITCH_UP)

    def arm_angular_yaw_left(self):
        self.actions.arm_cartesian(angular_amount=[0, 0, RobotController.ARM_ANGULAR_STEP])
        self._update_intervention_trace(InterventionActionMetadata.ARM_ANGULAR_YAW_LEFT)

    def arm_angular_yaw_right(self):
        self.actions.arm_cartesian(angular_amount=[0, 0, -RobotController.ARM_ANGULAR_STEP])
        self._update_intervention_trace(InterventionActionMetadata.ARM_ANGULAR_YAW_RIGHT)

    def arm_position_tuck(self):
        self.arm_position('tuck')

    def arm_position_ready(self):
        self.arm_position('ready')

    def arm_position(self, position):
        assert position in ["tuck", "ready"], "Unknown position: {}".format(position)
        self.actions.arm(poses="joint_poses.{}".format(position))
        self._update_intervention_trace(InterventionActionMetadata.ARM_POSITION, position=position)

    def crop_image(self):
        self.noop()
        self._update_intervention_trace(InterventionActionMetadata.CROP_IMAGE)

    def segment_image(self):
        self.noop()
        self._update_intervention_trace(InterventionActionMetadata.SEGMENT_IMAGE)

    def update_beliefs(self, beliefs):
        assert len(beliefs) == 1 and beliefs.keys()[0] in [
            BeliefKeys.DOOR_1_OPEN, BeliefKeys.CUBE_AT_PICKUP_1, BeliefKeys.CUBE_AT_DROPOFF
        ], "Unrecognized beliefs: {}".format(beliefs)
        self.actions.update_beliefs(beliefs=beliefs)
        self._update_intervention_trace(InterventionActionMetadata.UPDATE_BELIEFS, beliefs=beliefs)
