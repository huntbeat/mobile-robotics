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
from simple_plan import plan_path
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
CONTROL_PERIOD = rospy.Duration(0.01)
PATH = []
START = []
START_P = []

class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        rospy.init_node('deliver')
        rospy.loginfo('Initializing...')

	self.start_run = rospy.Publisher('/starting_point', String, queue_size=10)
	self.idx = 1

	self.begin = False
	self.move = False

	self.state = 'move'
        self.answer = None
	
	self.threshold = 25
	self.alpha = 0.05 
	self.beta = 0.10

	self.cur_x, self.cur_y, self.cur_d = (START_P[0], START_P[1], START_P[2] * -1 * np.pi + 0.0001)	

        self.start_particle = rospy.Subscriber('/start_particle', String,
                                                    self.update_callback)

        self.cmd_vel_pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist,
            queue_size=10)

        rospy.Timer(CONTROL_PERIOD, self.control_callback)


    def update_callback(self, msg):
	rospy.loginfo('Updating coordinates from pf!')
        data = json.loads(msg.data)
	rospy.loginfo(data)
        self.cur_x = float(data.pop(0) / PIXEL_METER)
        self.cur_y = float(-1 *data.pop(0) / PIXEL_METER)
        self.cur_d = float(data.pop(0))
	self.begin = True
	self.move = False

    def control_callback(self, timer_event=None):
        self.start_run.publish(String(data='FHN'))
	rospy.loginfo(self.state)
	
	if True:
    
	    cmd_vel = Twist()

	    goal_x = PATH[self.idx][0]
	    goal_y = PATH[self.idx][1]

	    if self.move == True:
	        cmd_vel.linear.x = 0.05
	        cmd_vel.linear.y = 0 
	        cmd_vel.angular.z = 0
	    
	    elif self.state == 'update':
	        self.idx += 1
	        if self.idx == len(PATH):
	    	    self.state = 'finish'
	        else:
	    	    self.state = 'calculate'

	    elif self.state == 'move':
	    	dif_x = goal_x - self.cur_x
	    	dif_y = self.cur_y - goal_y
		rospy.loginfo('Movement left')
		rospy.loginfo(dif_x)
		rospy.loginfo(dif_y)
		
		if abs(dif_x) < 40 and abs(dif_y) < 40:
		    self.state = 'update'
		    return
		elif abs(dif_y) < 20:
		    ratio = dif_x
		else:
		    ratio = dif_y
 	
	    	cmd_vel.linear.x = 0.010

	    elif self.state == 'calculate':
	    	dif_x = goal_x - self.cur_x
	    	dif_y = goal_y - self.cur_y

	    	if abs(dif_y) < 20 and dif_x > 0:
		    g_dir = 3
	    	elif abs(dif_y) < 20 and dif_x < 0:
		    g_dir = 1
	    	elif abs(dif_x) < 20 and dif_y > 0:
		    g_dir = 2
	    	else:
		    g_dir = 0

	        g_dir = np.pi/2.0 * g_dir	

		self.answer = g_dir

                self.state = 'rotate'

            elif self.state == 'rotate':
                movement = self.answer - (self.cur_d - (np.pi/2.0))
		rospy.loginfo('Amount of move left')
		rospy.loginfo(self.cur_d / np.pi * 180.0)

                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                cmd_vel.angular.z = 0.01

                if abs(movement) < 45.0 * np.pi / 180:
                    self.state = 'move'
	    			
	    elif self.state == 'finish':
	        cmd_vel.linear.x = 0
	    	cmd_vel.linear.y = 0 
	    	cmd_vel.angular.z = 0
	    	rospy.loginfo('Finished!')
	    	while True:
	    	    pass;
	    
	    rospy.loginfo(cmd_vel)
	    self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
	rospy.spin()

# main function
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("starting_node", help="Starting node name", type=str)
    parser.add_argument("destination_node", help="Destination node name", type=str)
    args = parser.parse_args()

    pprint.pprint(args)

    START = args.starting_node
    END = args.destination_node
	
    nodes = {}
    nodes['AOQ'] = (1000,700,0)
    nodes['CD'] = (1400,200,3)
    nodes['CK'] = (100,100,2)
    nodes['FHN'] = (300,700,0)
    nodes['JS'] = (1000,0,2)
    starting_x = nodes[START][0]
    starting_y = nodes[START][1]
    end_x = nodes[END][0]
    end_y = nodes[END][1]

    START_P = nodes[START]

    PATH = plan_path((starting_x,starting_y),(end_x,end_y))

    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
