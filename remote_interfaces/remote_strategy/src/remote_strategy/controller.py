#!/usr/bin/env python
# Provide an interface to the robot for remote assistance

from __future__ import print_function, division

import os
import sys
import time
import signal

import rospy

from assistance_msgs.msg import (RequestAssistanceResult, InterventionEvent,
                                 InterventionHypothesisMetadata,
                                 InterventionActionMetadata)
from assistance_msgs.srv import (EnableRemoteControl,
                                 EnableRemoteControlResponse,
                                 DisableRemoteControl,
                                 DisableRemoteControlResponse)
from std_srvs.srv import Trigger, TriggerResponse

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

        # TODO: Create and register the different subscribers. Also create a
        # global state flag that enables or disables this controller, and which
        # can be switched on from the remote server

    def start(self):
        self._app.run_server(host=RemoteController.APP_HOST,
                             port=RemoteController.APP_PORT,
                             debug=False)

    def stop(self, *args, **kwargs):
        # Give some time for rospy to shutdown
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def enable(self, req=None):
        self._current_error = req.request
        self._current_response = None
        return EnableRemoteControlResponse()

    def disable(self, req=None):
        self._current_error = None
        # This happens if the RViz window is closed and there is no recovery
        # strategy that is provided. Default is to then exit from the task
        if self._current_response is None:
            self._current_response = RequestAssistanceResult(resume_hint=RequestAssistanceResult.RESUME_NONE)
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
