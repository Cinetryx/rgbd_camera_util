#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TF Interactive marker
"""

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (InteractiveMarkerControl, InteractiveMarker,
    InteractiveMarkerFeedback)
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from threading import Lock


class TFMarkerServer():
    """TFMarkerServer"""
    def __init__(self, rate = 30.0):
        # Marker server
        self.server = InteractiveMarkerServer('camera_marker')
        # TF broadcaster
        self.tf = TransformBroadcaster()

        # Marker pose
        self.pose_mutex = Lock()
        self.marker_position = (0.0, 0.0, 0.0)
        self.marker_orientation = (0.0, 0.0, 0.0, 1.0)

        # Add marker
        self.add6DOF()

        # Timer for TF broadcaster
        rospy.Timer(rospy.Duration(1/rate), self.publish_transform)


    def add6DOF(self, init_position = Point( 0.0, 0.0, 0.0), frame_id = 'base_link'):
        marker = InteractiveMarker()
        marker.header.frame_id = frame_id
        marker.pose.position = init_position
        marker.scale = 0.3

        marker.name = 'camera_marker'
        marker.description = 'Camera 6-DOF pose control'

        # X axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # X axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Y axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # Y axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Z axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # Z axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Add marker to server
        self.server.insert(marker, self.marker_feedback)
        self.server.applyChanges()

    def publish_transform(self, timer_event):
        time = rospy.Time.now()
        self.pose_mutex.acquire()
        self.tf.sendTransform(self.marker_position, self.marker_orientation,
            time, 'moving_frame', 'base_link')
        self.pose_mutex.release()

    def marker_feedback(self, feedback):
        rospy.loginfo('Feedback from ' + feedback.marker_name)
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( 'Pose changed')
            # Update marker position
            self.pose_mutex.acquire()
            self.marker_position = (feedback.pose.position.x,
                feedback.pose.position.y, feedback.pose.position.z)
            # Update marker orientation
            self.marker_orientation = (feedback.pose.orientation.x,
                feedback.pose.orientation.y, feedback.pose.orientation.z,
                feedback.pose.orientation.w)
            self.pose_mutex.release()


if __name__ == "__main__":
    rospy.init_node('camera_tf')
    marker_server = TFMarkerServer()
    rospy.spin()
