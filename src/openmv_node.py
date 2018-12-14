#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import signal
import time

import cv2
import numpy as np
import rospy
import serial
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header


class OpenMVNode:
    """Reads images from OpenMV cam over USB and publishes them as ROS topic."""

    def __init__(self):

        rospy.init_node('openmv_node', anonymous=True)
        rospy.loginfo('openmv_node initialized.')

        # Serial connection
        self.baudrate = rospy.get_param('~baudrate', default=115200)
        self.device = rospy.get_param('~device', default='/dev/ttyACM0')
        self.timeout = 1
        self.rate = rospy.Rate(10)

        # Image params
        self.image_height = rospy.get_param('~image_height', default=128)
        self.image_width = rospy.get_param('~image_width', default=128)
        self.image_size = (self.image_height, self.image_width)
        self.image_decoder = rospy.get_param('~image_decoder', default='jpeg')
        self.image_mode = rospy.get_param('~image_mode', default='L') # also: RGB

        # ROS image transport
        self.bridge = CvBridge()
        self.use_compressed = rospy.get_param('~use_compressed', default=False)
        try:
            self.camera = serial.Serial(port=self.device, 
                                        baudrate=self.baudrate,
                                        timeout=self.timeout)
        except serial.serialutil.SerialException:
            rospy.logerr('Could not open device {}'.format(self.device))
            rospy.signal_shutdown('Could not open device: {}'.format(self.device))
            exit()

        self.data_class = CompressedImage if self.use_compressed else Image        
        self.image_topic = rospy.get_param('~image_topic', '/image_raw')

        # Conform to compressed image conventions
        if self.use_compressed:
            self.image_topic += '/compressed'
            rospy.set_param('~image_topic', self.image_topic)
        self.image_pub = rospy.Publisher(self.image_topic, self.data_class, queue_size=1)

        self.publish_images()

    def publish_images(self):

        while True:

            # self.rate.sleep()
            time.sleep(0.1)
            data = self.camera.read(self.camera.in_waiting)
            try:
                image = PILImage.frombuffer(self.image_mode,   
                                            self.image_size,
                                            data,
                                            self.image_decoder,
                                            self.image_mode,
                                            '')
            except ValueError:
                continue # disregard for now!

            image = np.asarray(image)

            # Create image message (regular or compressed)
            if self.use_compressed:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            else:
                image_msg = self.bridge.cv2_to_imgmsg(image)

            self.image_pub.publish(image_msg)
                

def shutdown_handle(signal, frame):
    exit()

def main():
    signal.signal(signal.SIGINT, shutdown_handle)
    OpenMVNode()

if __name__ == '__main__':
    main()
