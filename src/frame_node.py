#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class FrameNode:
    """Transforms poses from Motive Body VRPN server (NUE) to MAVROS (ENU)."""

    def __init__(self):

        rospy.init_node('frame_node', anonymous=True)
        rospy.loginfo('frame_node initialized.')

        # Position to be sent
        self.pos = PoseStamped()
        self.pos.header = Header()
        self.pos.header.frame_id = 'world'

        # From OptiTrack
        self.sub_topic = rospy.get_param('~vrpn_topic',
                                         default='/vrpn_client_node/drone/pose')
        self.pos_sub = rospy.Subscriber(self.sub_topic, 
                                        PoseStamped,
                                        callback=self.pos_callback,
                                        queue_size=1)

        # To PX4
        self.pub_topic = rospy.get_param('~mavros_topic',
                                          default='/mavros/vision_pose/pose')
        self.pos_pub = rospy.Publisher(self.pub_topic,
                                       PoseStamped,
                                       queue_size=1)

        rospy.spin()

    def pos_callback(self, pos_msg):

        self.pos.header.stamp = rospy.Time.now()

        # If Motive Body Up Axis = Y Up
        # Meaning VRPN ROS client frames are sent in NUE
        # Then PX4 receives UN-E
        # For PX4 to receive NED frames, we need the following conversion:
        #   x_frame = z_VRPN
        #   y_frame = x_VRPN
        #   z_frame = y_VRPN
        # https://github.com/ros-drivers/mocap_optitrack/blob/master/src/mocap_datapackets.cpp
        self.pos.pose.position.x = pos_msg.pose.position.z
        self.pos.pose.position.y = pos_msg.pose.position.x
        self.pos.pose.position.z = pos_msg.pose.position.y

        self.pos.pose.orientation.x = pos_msg.pose.orientation.z
        self.pos.pose.orientation.y = pos_msg.pose.orientation.x
        self.pos.pose.orientation.z = pos_msg.pose.orientation.y
        self.pos.pose.orientation.w = pos_msg.pose.orientation.w

        self.pos_pub.publish(self.pos)


def main():
    FrameNode()

if __name__ == '__main__':
    main()
