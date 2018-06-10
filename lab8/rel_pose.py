#T!/usr/bin/env python

import time
import math
import roslib
import rospy
import random
import numpy as np
import tf
import tf2_ros
import transform2d
import argparse
import pprint

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import ButtonEvent

import cv2
import json
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray as FloatArray
from std_msgs.msg import MultiArrayLayout as ArrayLayout
from std_msgs.msg import MultiArrayDimension as ArrayDimension
from std_msgs.msg import Float64 as Float
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

PIXEL_METER = 0.003048
CONTROL_PERIOD = rospy.Duration(1.0)

class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        rospy.init_node('rel_pose')
        rospy.loginfo('2. rel_pose.py : E28 Final Project')

	self.cur_pose = None
	self.start_pose = None
        self.rel_pose = None
	self.odom_pose = None

        self.publish_packet = FloatArray()
	self.t1 = 0.0
	self.t2 = 0.0

        self.lidar_info = None
	self.begin = False
	self.begin_transmit = False
        #arrays holds data that is (number_of_cylinders*2,1)

	#self.lidar = rospy.Subscriber('/lidar_scan', FloatArray, self.lidar_callback)
	self.lidar = rospy.Subscriber('/lidar_scan', String, self.lidar_callback)

	self.start = rospy.Subscriber('/starting_point', String, self.start_callback)

        self.odom = rospy.Subscriber('/odom',
                                     Odometry, self.odom_callback)

        #self.particle = rospy.Publisher('/particle', FloatArray, queue_size=10)
        self.particle = rospy.Publisher('/particle', String, queue_size=10)

	rospy.Timer(CONTROL_PERIOD, self.control_callback)

    def start_callback(self, msg):
	if self.begin == False:
	    self.begin = True
	    rospy.loginfo('It begins!')

    def lidar_callback(self, msg):
	rospy.loginfo('LaserScan sensor info received!')
        self.lidar_info = json.loads(msg.data)
	self.begin_transmit = True

    def odom_callback(self, msg):
	self.t1 = time.time()
	if self.begin:
            self.odom_pose = transform2d.transform2d_from_ros_pose(msg.pose.pose)
	    if not self.start_pose == None:
	        rospy.loginfo('Creating cur_pose from odom!')
	        self.cur_pose = self.odom_pose 
	    else:
		rospy.loginfo('Creating start_pose from odom!')
	        self.start_pose = self.odom_pose
	self.t2 = time.time()
	rospy.loginfo('TOTAL TIME: %s ------------------',self.t2-self.t1)

    def control_callback(self, timer_event=None):
        if not self.cur_pose == None and not self.start_pose == None and self.begin_transmit:
            rospy.loginfo('rel_pose calculation successful!')
            self.rel_pose = self.start_pose.inverse() * self.cur_pose
	    if not (abs(self.rel_pose.x) < 0.05 and abs(self.rel_pose.y) < 0.05 and abs(self.rel_pose.theta) < 0.1):
                toPublish = []
		toPublish = [self.rel_pose.x, self.rel_pose.y, self.rel_pose.theta]
                toPublish.append(self.lidar_info)
                self.particle.publish(json.dumps(toPublish))
		rospy.loginfo('Sending command to pf!')
                self.start_pose = self.cur_pose
	    else:
		rospy.loginfo('rel_pose too small, will try again...')

    # called by main function below (after init)
    def run(self):
        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
	rospy.spin()

# main function
if __name__ == '__main__':

    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
