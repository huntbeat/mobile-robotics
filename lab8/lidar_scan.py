#T!/usr/bin/env python

import time
import math
import rospy
import random
import numpy as np
import tf
import tf2_ros
import transform2d
import time
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
CYL_R = 0.044

class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        rospy.init_node("lidar_scan")
        rospy.loginfo('1. lider_scan.py : E28 Final Project')
        self.h_laser = None

        self.base_from_depth = None
        self.angles = None
        self.ranges = None
        self.points = None

        self.pts_x = None
        self.pts_y = None
        
        self.begin = False

	self.world_xyd = (0,0,0)
	self.world_xy = (0,0)
	self.world_pose = transform2d.Transform2D(x=self.world_xyd[0],y=self.world_xyd[1],
							theta=self.world_xyd[2]+0.000001)
	self.cylinder_info = []
	#self.cylinder_info = FloatArray()
	#self.cylinder_info.layout = ArrayLayout()
	#self.cylinder_info.layout.dim1 = ArrayDimension()
	#self.cylinder_info.layout.dim1.label = 'number'
	#self.cylinder_info.layout.dim1.size = 4
	#self.cylinder_info.layout.dim1.stride = 4*2
	#self.cylinder_info.layout.dim2.label = 'height_width'
	#self.cylinder_info.layout.dim2.size = 2
	#self.cylinder_info.layout.dim2.stride = 2

	self.t1 = 0.0
	self.t2 = 0.0

	self.max_x = 14 * 100 * PIXEL_METER + 0.05
	self.min_x = -0.05
	self.max_y = 0.05
	self.min_y = -7 * 100 * PIXEL_METER - 0.05

	self.lidar = rospy.Publisher('/lidar_scan', String, queue_size=10)
	#self.lidar = rospy.Publisher('/lidar_scan', FloatArray, queue_size=10)

        self.start_run = rospy.Subscriber('/starting_point', String, self.start_callback)

        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.start_particle = rospy.Subscriber('/start_particle', String, 
                                  self.update_callback)

    def start_callback(self, msg):
	if self.begin == False:
	    rospy.loginfo('Received node info from delivery!')
	    start = str(msg.data)
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
            self.begin = True
	    rospy.loginfo('It begins!')

    def update_callback(self, msg):
	data = json.loads(msg.data)
        starting_x = float(data.pop(0))
        starting_y = float(data.pop(0))
        starting_d = float(data.pop(0))
        INPUT = (starting_x*PIXEL_METER, -1*starting_y*PIXEL_METER, -1*starting_d*np.pi/2)
	self.cur_xyd = INPUT
	self.cur_xy = (self.cur_xyd[0],self.cur_xyd[1])
	self.cur_pose = transform2d.Transform2D(x=self.cur_xyd[0],y=self.cur_xyd[1],
							theta=self.cur_xyd[2]+0.000001)

    def scan_callback(self, msg):
	self.t1 = time.time()
        if self.begin == True:
	    rospy.loginfo('LaserScan sensor processing...')
            count = len(msg.ranges)
            self.angles = (msg.angle_min + np.arange(count, dtype=float) * msg.angle_increment)
            self.ranges = np.array(msg.ranges)
            self.pts_y = self.ranges * np.cos(self.angles)
            self.pts_x = -1 * self.ranges * np.sin(self.angles)
	    self.find_cylinder()

    def find_cylinder(self):
	# It is required that we have walls surround the area, as all NaN values will be ignored
	#self.cylinder_info.data = []
	self.cylinder_info = []
	number = 0
	cylinder_found = False
	for idx in range(0,len(self.ranges)):
	    if not np.isnan(self.ranges[idx]):
		laser_x = self.pts_x[idx]
		laser_y = self.pts_y[idx]
		laser_ang = self.angles[idx]
		laser_pose = transform2d.Transform2D(x=laser_x,y=laser_y,theta=laser_ang)
		robot_to_laser = transform2d.Transform2D(x=0.00001,y=-0.10,theta=-0.00001)
		world_to_obj = self.cur_pose * robot_to_laser * laser_pose
		w_obj_x = world_to_obj.x
		w_obj_y = world_to_obj.y
		w_obj_theta = world_to_obj.theta
		if (w_obj_x < self.max_x and w_obj_x > self.min_x and
			w_obj_y < self.max_y and w_obj_y > self.min_y
			and not cylinder_found):
		    right_index = idx
		    cylinder_found = True
		if ((w_obj_x > self.max_x or w_obj_x < self.min_x or
			w_obj_y > self.max_y or w_obj_y < self.min_y)
			and cylinder_found):
		    left_index = idx
		    cylinder_found = False
		    true_index = (left_index + right_index) / 2	
		    true_distance = 0
		    hello = 0
		    for number in range(true_index-2,true_index+3):
			if not np.isnan(self.ranges[number]): 
		            true_distance += self.ranges[number]
			    hello += 1.0
		    true_distance = true_distance / hello
		    true_distance += CYL_R
		    true_ang = self.angles[true_index]
		    true_y = true_distance * np.cos(true_ang)
		    true_x = -1 * true_distance * np.sin(true_ang)
		    true_pose = transform2d.Transform2D(x=true_x,y=true_y,theta=true_ang)
		    real_true = robot_to_laser * true_pose
		    real_distance = np.sqrt(real_true.x**2 + real_true.y**2)
		    real_ang = real_true.theta
		    #self.cylinder_info.data.append(Float(data=real_ang))
		    #self.cylinder_info.data.append(Float(data=real_distance))
		    self.cylinder_info.append((real_ang,real_distance))
		    number += 1
	self.lidar.publish(json.dumps(self.cylinder_info))
	rospy.loginfo(self.cylinder_info)
	#self.lidar.publish(self.cylinder_info)
	rospy.loginfo('Published to rel_pose node!')
	self.t2 = time.time()
	rospy.loginfo('TOTAL TIME: %s ------------------------------', self.t2-self.t1)

    # called by main function below (after init)
    def run(self):
        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")

        ####################
        # Add vision support
        cv2.destroyAllWindows()
        ####################


# main function
if __name__ == '__main__':

    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
