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
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry as laser

from simple_plan import plan_path  

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

PIXEL_METER = 0.003048
INPUT = []   

class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        rospy.loginfo('2. rel_pose.py : E28 Final Project')
        rospy.init_node('rel_pose')

	self.cur_pose = None
	self.start_pose = None
	self.odom_pose = None

        self.begin = False

	self.lidar = rospy.Subscriber('/lidar_scan', FloatArray, self.lidar_callback)

        self.starting_point = rospy.Subscriber('/starting_point', String, self.start_callback)

        self.odom = rospy.Subscriber('odom',
                                     Odometry, self.odom_callback)

        self.particle = rospy.Publisher('/particle', FloatArray, queue_size=10)
        self.start_particle = rospy.Subscriber('/start_particle', String, self.start_particle)

    def start_callback(self, msg):
	start = msg.data
        self.begin = True
	nodes = {}
	nodes['AOQ'] = (1000,700,0)
	nodes['CD'] = (1400,200,3)
	nodes['CK'] = (100,100,2)
	nodes['FHN'] = (300,700,0)
	nodes['JS'] = (1000,0,2)
        starting_x = nodes[start][0]
        starting_y = nodes[start][1]
        starting_d = nodes[start][2]
        INPUT = (starting_x*PIXEL_METER, -1*starting_y*PIXEL_METER, -1*starting_d*np.pi/2)
	self.cur_xyd = INPUT
	self.cur_xy = (self.cur_xyd[0],self.cur_xyd[1])
	self.cur_pose = transform2d.Transform2D(x=self.cur_xyd[0],y=self.cur_xyd[1],
							theta=self.cur_xyd[2]+0.000001)

    def bumper_callback(self, data):
        rospy.loginfo('Got a bump!')
        self.is_bumped = True
	rospy.loginfo('bumper call: %s seconds', time.time() - t0)

    def get_current_pose(self):
        try:
            ros_xform = self.odom_listener.lookupTransform(
                '/odom', '/base_footprint', rospy.Time(0))
            return transform2d.transform2d_from_ros_transform(ros_xform)
        except tf.LookupException:
            return None

    # get /odom messages and convert to a transform2d
    def odom_callback(self, msg):
        self.odom_pose = transform2d.transform2d_from_ros_pose(msg.pose.pose)
	self.cur_pixel = transform2d.Transform2D(x=self.cur_xyd[0],
					y=self.cur_xyd[1],
					theta=self.cur_xyd[2])
	self.start_pixel = transform2d.Transform2D()
	self.cur_pose = transform2d.Transform2D(x=PIXEL_METER*self.cur_xyd[0],
					y=PIXEL_METER*self.cur_xyd[1],
					theta=self.cur_xyd[2])
	self.start_pose = transform2d.Transform2D()
	self.rel_pixel = self.cur_pixel * self.start_pixel.inverse()
	self.rel_pose = self.cur_pose * self.start_pose.inverse()

    # called by main function below (after init)
    def run(self):
        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
	rospy.spin()

# main function
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("sx", help="Starting position in x", type=int)
    parser.add_argument("sy", help="Starting position in y", type=int)
    parser.add_argument("sd", help="Starting orientation, from 0 to 3", type=int)
    args = parser.parse_args()

    pprint.pprint(args)

    INPUT = (args.sx, args.sy, args.sd*np.pi/2+0.00001)

    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
