#!/usr/bin/env python
# Provide an interface to the robot for remote assistance

from __future__ import print_function, division

import os
import sys
import time
import signal

import rospy

# Plotly and Dash
import plotly.graph_objs as go
import dash
import dash_core_components as dcc
import dash_html_components as html


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

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP

        # Initialize the application
        self._define_app()

        # Create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

    def start(self):
        self._app.run_server(host=RemoteController.APP_HOST,
                             port=RemoteController.APP_PORT,
                             debug=False)

    def stop(self, *args, **kwargs):
        # Give some time for rospy to shutdown
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def _define_app(self):
        """
        Define the app to control the robot.
        """
        self._app.layout = html.Div([dcc.Markdown("Hello World!")])

