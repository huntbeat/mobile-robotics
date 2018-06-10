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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

from matplotlib import pyplot as plt

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)
PLOT_PERIOD = rospy.Duration(0.2)

INPUT = []
PIXEL_METER = 0.003048
CYL_R = 0.044

class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        
        rospy.loginfo('E28 Mobile Robotics - Final Project!')
        rospy.init_node("lidarscan")
        self.h_laser = None

        self.base_from_depth = None
        self.angles = None
        self.ranges = None
        self.points = None

        self.pts_x = None
        self.pts_y = None

	self.world_xyd = (0,0,0)
	self.world_xy = (0,0)
	self.world_pose = transform2d.Transform2D(x=self.world_xyd[0],y=self.world_xyd[1],
							theta=self.world_xyd[2]+0.000001)
	
	self.cur_xyd = INPUT 
	self.cur_xy = (self.cur_xyd[0],self.cur_xyd[1])
	self.cur_pose = transform2d.Transform2D(x=self.cur_xyd[0],y=self.cur_xyd[1],
							theta=self.cur_xyd[2]+0.000001)

	self.cylinder_info = None

	self.odom_0 = None
	self.odom_1 = None
	self.rel_pose = None

        self.is_bumped = False
	
	self.max_x = 14 * 100 * PIXEL_METER + 0.05
	self.min_x = 0.05
	self.max_y = 0.05
	self.min_y = -8 * 100 * PIXEL_METER - 0.05

        self.sensors = rospy.Subscriber('/mobile_base/sensors/core',
						SensorState, self.sensor_callback)

        # Laser scan emulation support
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def button_callback(self,data):
        if data.state == ButtonEvent.RELEASED:
            state = "released"
        else:
            state = "pressed"

        if data.button == ButtonEvent.Button0:
            button = "B0"
        elif data.button == ButtonEvent.Button1:
            button = "B1"
        else:
            button = "B2"
        rospy.loginfo("Button %r: %s"%(button, state))

    # Disable this method for grabbing bumper state
    # called whenever sensor messages are received
    def sensor_callback(self, msg):
        if msg.bumper:
            rospy.loginfo('Got a bump!!')
            self.is_bumped = True

        if msg.cliff:
            rospy.loginfo('quitting because picked up/bumped')
            rospy.signal_shutdown('safety stop')

    def bumper_callback(self, data):
        rospy.loginfo('Got a bump!')
        self.is_bumped = True

    # get /odom messages and convert to a transform2d
    def odom_callback(self, msg):
	if self.odom_0 == None:
		self.odom_0 = transform2d.transform2d_from_ros_pose(msg.pose.pose)
	else:
	    self.odom_1 = transform2d.transform2d_from_ros_pose(msg.pose.pose)
	    self.rel_pose = self.odom_1 * self.odom_0.inverse()
	    self.odom_0 = self.odom_1

    # called 100 times per second
    def control_callback(self, timer_event=None):
        cmd_vel = Twist()
        # Publish the command and save the previous output
        self.cmd_vel_pub.publish(cmd_vel)
       
    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)
        cv2.imshow("Image window", cv_image)
	while cv2.waitKey(3) > 0: pass;

    def plot_callback(self, timer_event=None):
        if self.pts_x is None:
            return
        if self.h_laser == None:
            self.h_laser, = plt.plot(self.pts_x, self.pts_y)
            plt.axis([-3.0, 3.0, -1.5, 3])
            plt.ion()
            plt.show()
            rospy.loginfo("initializing plot")
        else:
            self.h_laser.set_xdata(self.pts_x)
            self.h_laser.set_ydata(self.pts_y)
            plt.draw()
            plt.pause(0.01)
            rospy.loginfo("redrawing plot")

    def scan_callback(self, msg):
        count = len(msg.ranges)
        self.angles = (msg.angle_min + np.arange(count, dtype=float) * msg.angle_increment)
        self.ranges = np.array(msg.ranges)
        self.pts_y = self.ranges * np.cos(self.angles)
        self.pts_x = -1 * self.ranges * np.sin(self.angles)
	self.cylinder_info = self.find_cylinder()

    def find_cylinder(self):
	# It is required that we have walls surround the area, as all NaN values will be ignored
	cylinders = []
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
		    true_distance = self.ranges[true_index] + CYL_R
		    true_ang = self.angles[true_index]
		    true_y = true_distance * np.cos(true_ang)
		    true_x = -1 * true_distance * np.sin(true_ang)
		    true_pose = transform2d.Transform2D(x=true_x,y=true_y,theta=true_ang)
		    real_true = robot_to_laser * true_pose
		    real_distance = np.sqrt(real_true.x**2 + real_true.y**2)
		    real_ang = real_true.theta
		    cylinders.append((real_ang,real_distance))
	return cylinders

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

    parser = argparse.ArgumentParser()
    parser.add_argument("sx", help="Starting position in x", type=int)
    parser.add_argument("sy", help="Starting position in y", type=int)
    parser.add_argument("sd", help="Starting orientation, from 0 to 3", type=int)
    args = parser.parse_args()

    pprint.pprint(args)

    INPUT = (args.sx*PIXEL_METER, -1*args.sy*PIXEL_METER, -1*args.sd*np.pi/2)

    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
