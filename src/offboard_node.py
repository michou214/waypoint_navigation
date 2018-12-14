#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy

from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, ExtendedState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Point


class OffboardNode:
    """Sends the same pose setpoint repeatedly to test offboard mode."""

    OFFBOARD = 'OFFBOARD'

    def __init__(self):

        rospy.init_node('offboard_node', anonymous=True)
        rospy.loginfo('offboard_node initialized.')

        # Arming client
        self.arming_service = rospy.get_param('~arming_topic',
                                              'mavros/cmd/arming')
        rospy.wait_for_service(self.arming_service)
        self.arming_client = rospy.ServiceProxy(self.arming_service,
                                                CommandBool)

        # Set mode client
        self.mode_service = rospy.get_param('~mode_topic',
                                            'mavros/set_mode')
        rospy.wait_for_service(self.mode_service)
        self.mode_client = rospy.ServiceProxy(self.mode_service, SetMode)

        # Set up position setpoint
        self.pose = PoseStamped()
        self.pose.header = Header()
        self.pose.pose.position = Point(0, 0, 0.5)  # ENU

        self.vel = TwistStamped()
        self.vel.header = Header()
        self.vel.twist.linear = Point(0, 0, -0.3)  # ENU

        # Set up pose setpoint publisher
        self.rate = rospy.Rate(20)
        self.setpoint_pose_topic = rospy.get_param('~setpoint_topic',
                                                   'mavros/setpoint_position/local')
        self.setpoint_pose_pub = rospy.Publisher(self.setpoint_pose_topic,
                                                 PoseStamped,
                                                 queue_size=1)

        self.setpoint_vel_topic = rospy.get_param('~setpoint_vel_topic',
                                                  'mavros/setpoint_velocity/cmd_vel')
        self.setpoint_vel_pub = rospy.Publisher(self.setpoint_vel_topic,
                                                TwistStamped,
                                                queue_size=1)

        # State and extended state subscribers
        self.state_topic = rospy.get_param('~state_topic',
                                           default='mavros/state')
        self.state_sub = rospy.Subscriber(self.state_topic, State,
                                          callback=self.state_callback,
                                          queue_size=1)
        self.ext_state_topic = rospy.get_param('~extended_state_topic',
                                               'mavros/extended_state')
        self.ext_state_sub = rospy.Subscriber(self.ext_state_topic,
                                              ExtendedState,
                                              callback=self.ext_state_callback,
                                              queue_size=1)
        self.state = State()  # everything False by default!
        self.ext_state = ExtendedState()

        self.takeoff_hold_land()

    def state_callback(self, state_msg):
        self.state = state_msg

    def ext_state_callback(self, ext_state_msg):
        self.ext_state = ext_state_msg

    def takeoff_hold_land(self):

        last_request = rospy.Time.now()
        published_poses = 0
        started_landing = False
        finished_hovering = False

        while not rospy.is_shutdown():

            # Switch to offboard mode and arm
            has_waited = rospy.Time.now() - last_request > rospy.Duration(5)
            if self.state.connected and published_poses > 100 and has_waited:
                if self.state.mode != OffboardNode.OFFBOARD:
                    resp = self.mode_client(custom_mode=OffboardNode.OFFBOARD)
                    if resp.mode_sent:
                        rospy.loginfo('Offboard mode enabled.')
                    last_request = rospy.Time.now()
                elif not self.state.armed:
                    resp = self.arming_client(value=True)
                    if resp.success:
                        rospy.loginfo('Vehicle armed. Taking off.')
                        rospy.loginfo('Hovering for 10 seconds.')
                    last_request = rospy.Time.now()

            has_landed = self.ext_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
            if has_landed and self.state.armed and finished_hovering:
                resp = self.arming_client(value=False)
                if resp.success:
                    rospy.loginfo('Vehicle landed. Exiting.')
                    break

            has_hovered = rospy.Time.now() - last_request > rospy.Duration(10)
            if has_hovered:
                finished_hovering = True
                if not started_landing:
                    rospy.loginfo('Hovered for 10 seconds. Landing.')
                    started_landing = True
                self.vel.header.stamp = rospy.Time.now()
                self.setpoint_vel_pub.publish(self.vel)
                self.rate.sleep()
                continue

            self.pose.header.stamp = rospy.Time.now()
            self.setpoint_pose_pub.publish(self.pose)
            published_poses += 1
            self.rate.sleep()


def main():
    OffboardNode()


if __name__ == '__main__':
    main()
