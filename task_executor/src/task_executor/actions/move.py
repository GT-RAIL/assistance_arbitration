#!/usr/bin/env
# The move action in a task plan

from __future__ import print_function, division

from math import sin, cos

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from task_executor.msg import Waypoint
from task_executor.srv import GetWaypoints


class MoveAction(AbstractStep):
    """Move to a location"""

    def init(self):
        self._move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._get_waypoints_srv = rospy.ServiceProxy("database/waypoints", GetWaypoints)

        rospy.loginfo("Connecting to move_base...")
        self._move_base_client.wait_for_server()
        rospy.loginfo("...move_base connected")

        rospy.loginfo("Connecting to database services...")
        self._get_waypoints_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, location):
        # Parse out the waypoints
        coords = self._parse_location(location)
        if coords is None:
            error_msg = "Unknown format for location: {}".format(location)
            rospy.logerr(error_msg)
            yield self.set_aborted(exception=Exception(error_msg))
            raise StopIteration()

        rospy.logdebug("Moving to location(s): {}".format(coords))

        status = GoalStatus.LOST
        for coord in coords:
            rospy.loginfo("Going to coordinate: {}".format(coord))

            # Create and send the goal
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = coord.x
            goal.target_pose.pose.position.y = coord.y
            goal.target_pose.pose.orientation.z = sin(coord.theta/2.0)
            goal.target_pose.pose.orientation.w = cos(coord.theta/2.0)
            goal.target_pose.header.frame_id = coord.frame
            goal.target_pose.header.stamp = rospy.Time.now()
            self._move_base_client.send_goal(goal)

            # Yield running while the move_client is executing
            while self._move_base_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status and stop executing if we didn't complete our goal
            status = self._move_base_client.get_state()
            if status != GoalStatus.SUCCEEDED:
                break

        # Finally, yield based on the final status
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted()
        else:
            yield self.set_aborted()

    def stop(self):
        self._move_base_client.cancel_goal()

    def _parse_location(self, location):
        coords = None
        if type(location) == str:
            db_name, location = location.split('.', 1)
            if db_name == 'locations':
                coords = self._get_waypoints_srv(location).waypoints
        elif type(location) == dict:
            coords = [Waypoint(**location),]
        elif type(location) == list or type(location) == tuple:
            coords = [Waypoint(**x) for x in location]

        return coords
