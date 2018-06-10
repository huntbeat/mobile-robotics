#!/usr/bin/env python

import math
import rospy
import random
import numpy
import tf
import tf2_ros
import transform2d
import time

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

# Our angular control limits
ANGULAR_TOL = 0.01  # rad (about 1/2 degree)
ANGULAR_RAMP_TIME = 2.0  # s
ANGULAR_RAMP_SPEED = 1.5  # rad/s
ANGULAR_MIN_SPEED = 0.2  # rad/s
ANGULAR_GAIN = 3.0  # rad/s per rad, so actually just 1/s

class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        
        rospy.loginfo('Initializing Test Environment')
        rospy.init_node("kobuki_button")
        self.h_laser = None

        self.base_from_depth = None
        self.angles = None
        self.ranges = None
        self.points = None

        self.pts_x = None
        self.pts_y = None

        self.is_bumped = False

        # Set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist,
            queue_size=10)

        # set up control timer at 100 Hz for publishing
        rospy.Timer(CONTROL_PERIOD, self.control_callback)
        rospy.Timer(PLOT_PERIOD, self.plot_callback)

        rospy.Subscriber(
            "/mobile_base/events/button",
            ButtonEvent, self.button_callback)

        self.sensors = rospy.Subscriber(
            '/mobile_base/sensors/core',
            SensorState, self.sensor_callback)
        # Subscribe directly to bumper measurements
        self.bumper = rospy.Subscriber(
            '/mobile_base/events/bumper',
            BumperEvent, self.bumper_callback)

        # Subscribe directly to odometry messages
        self.odom = rospy.Subscriber('/odom',
                                     Odometry, self.odom_callback)

        ####################
        # Add vision support
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",
            Image, self.camera_callback)
        ####################

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
        self.cur_pose = transform2d.transform2d_from_ros_pose(msg.pose.pose)
        #rospy.loginfo("Current Pose:")
        #rospy.loginfo(self.cur_pose)

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
        cv2.waitKey(3)

    def plot_callback(self, timer_event=None):
        if self.pts_x is None:
            return
        if self.h_laser == None:
            self.h_laser, = plt.plot(self.pts_x, self.pts_y)
            plt.axis([-1.5, 1.5, 0, 3])
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
        self.angles = (msg.angle_min + numpy.arange(count, dtype=float) * msg.angle_increment)
        self.ranges = numpy.array(msg.ranges)
        self.pts_y = self.ranges * numpy.cos(self.angles)
        self.pts_x = -1 * self.ranges * numpy.sin(self.angles)

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

