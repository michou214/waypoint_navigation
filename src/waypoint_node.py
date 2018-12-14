#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class WaypointNode:
    """Implements waypoint navigation."""

    def __init__(self):

        rospy.init_node('waypoint_node', anonymous=True)
        rospy.loginfo('waypoint_node initialized.')

        self.pose_msg = PoseStamped()
        self.pose_msg.header = Header()
        self.pose_msg.header.frame_id = 'world'

        self.rate = rospy.Rate(10) # Hz

        # When vision_pose topic is used, nothing is published on local_position
        self.pose_topic = rospy.get_param('~pose_topic',
                                          #default='/mavros/local_position/pose')
                                          default='/mavros/vision_pose/pose')
        self.pose_sub = rospy.Subscriber(self.pose_topic,
                                         PoseStamped,
                                         callback=self.pose_callback,
                                         queue_size=1)

        self.setpoint_topic = rospy.get_param('~setpoint_topic',
                                              default='/mavros/setpoint_position/local')
        self.setpoint_pub = rospy.Publisher(self.setpoint_topic,
                                            PoseStamped,
                                            queue_size=1)

        self.hover_alt = rospy.get_param('~hover_alt', default=2.0) # meters

        # Everything in ENU frame
        self.waypoints = [
            np.array([ 1,  1, self.hover_alt]),
            np.array([ 1, -1, self.hover_alt]),
            np.array([-1, -1, self.hover_alt]),
            np.array([-1,  1, self.hover_alt]),
        ]
        self.waypoint_index = 0
        self.acceptance_rad = rospy.get_param('~acceptance_rad', default=0.2) # meters

        self.pos = None
        self.waypoint = self.waypoints[self.waypoint_index]

        rospy.spin()

    def pose_callback(self, pose_msg):

        p = pose_msg.pose.position
        self.pos = np.array([p.x, p.y, p.z])

        # Check if waypoint is reached
        distance = np.linalg.norm(self.pos - self.waypoint)
        if distance < self.acceptance_rad:

            rospy.loginfo('Waypoint reached: {}'.format(self.waypoint))

            # Cycle through waypoints
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.waypoint_index = 0

            self.waypoint = self.waypoints[self.waypoint_index]
            
            rospy.loginfo('Next waypoint: {}'.format(self.waypoint))
        
        self.pose_msg.pose.position.x = self.waypoint[0]
        self.pose_msg.pose.position.y = self.waypoint[1]
        self.pose_msg.pose.position.z = self.waypoint[2]

        self.setpoint_pub.publish(self.pose_msg)

        try:
            self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


def main():
    WaypointNode()

if __name__ == '__main__':
    main()
