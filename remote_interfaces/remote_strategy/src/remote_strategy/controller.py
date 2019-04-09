#!/usr/bin/env python
# Provide an interface to the robot for remote assistance

from __future__ import print_function, division

import os
import sys
import time
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

        # Initialize the application
        self._define_app()

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
        self.controller = RobotController(get_default_actions())

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
             dcc.Markdown('', className='', id='failure-information')] +
            [html.H3('Fault Hypotheses', className='row')] +
            [html.Div(
                [
                    html.Div(dcc.Dropdown(id='hypothesis_{}'.format(idx),
                                          options=Annotations.RESULT_OPTIONS,
                                          value=None,
                                          className="dropdown form-control-sm"),
                             className='col-8'),
                    html.Div([
                        dcc.Input(type='checkbox',
                                  id='hypothesis_{}_certain'.format(idx),
                                  value=False,
                                  className="form-check-input"),
                        dcc.Markdown("Confirmed", className='form-check-label'),
                    ], className='form-check col-4'),
                ],
                className='row my-4')
             for idx in xrange(RemoteController.MAX_NUM_HYPOTHESES)],
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

        # The final layout of the interface
        self._app.layout = html.Div([hypothesis_layout, actions_layout])

        # Then register callbacks for each of the buttons
        # TODO: Requires setup of the ROS system

    def _on_relocalize(self, msg):
        """Relocalization action taken on RViz"""
        event_msg = InterventionEvent(stamp=msg.header.stamp,
                                      type=InterventionEvent.ACTION_EVENT)
        event_msg.action_metadata.type = InterventionActionMetadata.RELOCALIZE
        event_msg.action_metadata.args = pickle.dumps(msg)
        self._trace_pub.publish(event_msg)

    def _on_move_goal(self, msg):
        """Move base goal provided on RViz"""
        event_msg = InterventionEvent(stamp=msg.header.stamp,
                                      type=InterventionEvent.ACTION_EVENT)
        event_msg.action_metadata.type = InterventionActionMetadata.MOVE_WAYPOINT
        event_msg.action_metadata.args = pickle.dumps(msg)
        self._trace_pub.publish(event_msg)


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

    def __init__(self, actions):
        self._enabled = False

        # First get the actions that have been defined in the task_executor
        self.actions = actions

        # Setup the subscribers to monitor the robot state, as necessary

    def start(self):
        # self.actions.init()
        pass

    def stop(self):
        pass

    def enable(self):
        self._enabled = True

    def disable(self):
        self._disabled = False

    def look_up(self):
        self.actions.look_pan_tilt(tilt_amount=-RobotController.LOOK_TILT_STEP)

    def look_down(self):
        self.actions.look_pan_tilt(tilt_amount=RobotController.LOOK_TILT_STEP)

    def look_left(self):
        self.actions.look_pan_tilt(pan_amount=RobotController.LOOK_PAN_STEP)

    def look_right(self):
        self.actions.look_pan_tilt(pan_amount=-RobotController.LOOK_PAN_STEP)

    def move_forward(self):
        self.actions.move_planar(linear_amount=RobotController.MOVE_LINEAR_STEP)

    def move_backward(self):
        self.actions.move_planar(linear_amount=-RobotController.MOVE_LINEAR_STEP)

    def move_left(self):
        self.actions.move_planar(angular_amount=RobotController.MOVE_ANGULAR_STEP)

    def move_right(self):
        self.actions.move_planar(angular_amount=-RobotController.MOVE_ANGULAR_STEP)

    def torso_up(self):
        self.actions.torso_linear(amount=RobotController.TORSO_STEP)

    def torso_down(self):
        self.actions.torso_linear(amount=-RobotController.TORSO_STEP)

    def arm_linear_up(self):
        self.actions.arm_cartesian(linear_amount=[0, 0, RobotController.ARM_LINEAR_STEP])

    def arm_linear_down(self):
        self.actions.arm_cartesian(linear_amount=[0, 0, -RobotController.ARM_LINEAR_STEP])

    def arm_linear_left(self):
        self.actions.arm_cartesian(linear_amount=[0, -RobotController.ARM_LINEAR_STEP, 0])

    def arm_linear_right(self):
        self.actions.arm_cartesian(linear_amount=[0, RobotController.ARM_LINEAR_STEP, 0])

    def arm_linear_forward(self):
        self.actions.arm_cartesian(linear_amount=[RobotController.ARM_LINEAR_STEP, 0, 0])

    def arm_linear_backward(self):
        self.actions.arm_cartesian(linear_amount=[-RobotController.ARM_LINEAR_STEP, 0, 0])

    def arm_angular_roll_left(self):
        self.actions.arm_cartesian(angular_amount=[RobotController.ARM_ANGULAR_STEP, 0, 0])

    def arm_angular_roll_right(self):
        self.actions.arm_cartesian(angular_amount=[-RobotController.ARM_ANGULAR_STEP, 0, 0])

    def arm_angular_pitch_down(self):
        self.actions.arm_cartesian(angular_amount=[0, RobotController.ARM_ANGULAR_STEP, 0])

    def arm_angular_pitch_up(self):
        self.actions.arm_cartesian(angular_amount=[0, -RobotController.ARM_ANGULAR_STEP, 0])

    def arm_angular_yaw_left(self):
        self.actions.arm_cartesian(angular_amount=[0, 0, RobotController.ARM_ANGULAR_STEP])

    def arm_angular_yaw_right(self):
        self.actions.arm_cartesian(angular_amount=[0, 0, -RobotController.ARM_ANGULAR_STEP])

    def arm_position(self, position):
        assert position in ["tuck", "ready"], "Unknown position: {}".format(position)
        self.actions.arm(poses="joint_poses.{}".format(position))

    def update_beliefs(self, beliefs):
        assert len(beliefs) == 1 and beliefs.keys()[0] in [
            BeliefKeys.DOOR_1_OPEN, BeliefKeys.CUBE_AT_PICKUP_1, BeliefKeys.CUBE_AT_DROPOFF
        ], "Unrecognized beliefs: {}".format(beliefs)
        self.actions.update_beliefs(beliefs=beliefs)
