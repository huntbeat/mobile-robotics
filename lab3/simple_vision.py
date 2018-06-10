#!/usr/bin/env python

import math
import roslib
import rospy
import random
import numpy
import tf
import tf2_ros
import transform2d

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent

####################
# Add vision support
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
####################

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# Our angular control limits
ANGULAR_TOL = 0.01  # rad (about 1/2 degree)
ANGULAR_RAMP_TIME = 2.0  # s
ANGULAR_RAMP_SPEED = 1.5  # rad/s
ANGULAR_MIN_SPEED = 0.2  # rad/s
ANGULAR_GAIN = 3.0  # rad/s per rad, so actually just 1/s


class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        rospy.loginfo('Initializing Simple movement')

        # initialize rospy
        rospy.init_node('simple')

        # May be good to keep track of timing events
        self.start_time = rospy.get_rostime()
        self.start_pose = None

        # Add a notion of a state machine
        self.state = 'approach'
        self.is_bumped = False

        # Set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist,
            queue_size=10)

        # set up control timer at 100 Hz for publishing
        rospy.Timer(CONTROL_PERIOD, self.control_callback)

        # Two ways to access numper information
        # Set up subscriber for sensor data of bumpers/cliffs
        self.sensors = rospy.Subscriber('/mobile_base/sensors/core',
                                        SensorState, self.sensor_callback)
        # Subscribe directly to bumper measurements
        self.bumper = rospy.Subscriber('/mobile_base/events/bumper',
                                       BumperEvent, self.bumper_callback)

        # Two ways to access odometry information
        # set up a TransformListener to get odometry information
        # self.odom_listener = tf.TransformListener()
        # Subscribe directly to odometry messages
        self.odom = rospy.Subscriber('odom',
                                     Odometry, self.odom_callback)

        ####################
        # Add vision support
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        ####################

    # Disable this method for grabbing bumper state
    # called whenever sensor messages are received
    def sensor_callback(self, msg):
        if msg.bumper:
            rospy.loginfo('Got a bump!!')
        if msg.cliff:
            rospy.loginfo('quitting because picked up/bumped')
            rospy.signal_shutdown('safety stop')

    def bumper_callback(self, data):
        rospy.loginfo('Got a bump!')
        self.is_bumped = True

    # Disable this method for grabbing odom pose
    # get current pose from TransformListener and convert it into a transform2d
    def get_current_pose(self):
        try:
            ros_xform = self.odom_listener.lookupTransform(
                '/odom', '/base_footprint', rospy.Time(0))
            return transform2d.transform2d_from_ros_transform(ros_xform)
        except tf.LookupException:
            return None

    # get /odom messages and convert to a transform2d
    def odom_callback(self, msg):
        self.cur_pose = transform2d.transform2d_from_ros_pose(msg.pose.pose)
        rospy.loginfo(self.cur_pose)

    # Angular Control (possibly useful)
    def compute_turn_vel(self, rel_pose, desired_angle, time):
        # Proportional Gain of a PID controller
        # We will discuss in class :)
        angle_error = desired_angle - rel_pose.theta
        angular_vel = ANGULAR_GAIN * angle_error

        # Ask for a minimum speed
        clamped_vel = max(abs(angular_vel), ANGULAR_MIN_SPEED)
        sign = numpy.sign(angular_vel)
        angular_vel = sign * clamped_vel

        # For how long will we be commanding this velocity
        effective_time = min(time, ANGULAR_RAMP_TIME)
        vmax = effective_time * ANGULAR_RAMP_SPEED / ANGULAR_RAMP_TIME

        # Do not exceed our speed limit
        angular_vel = numpy.clip(angular_vel, -vmax, vmax)

        return angular_vel, angle_error

    # called 100 times per second
    def control_callback(self, timer_event=None):

        # Update state
        if self.is_bumped:
            self.state = 'turnaround'

        # Deprecated method for obtaining pose
        # self.cur_pose = self.get_current_pose()

        # Make some logic for moving
        if self.cur_pose is not None:
            if self.start_pose is None:
                self.start_pose = self.cur_pose.copy()
            rel_pose = self.start_pose.inverse() * self.cur_pose
        else:
            rel_pose = None

        # Initialize commanded vel
        cmd_vel = Twist()

        # Possibly useful
        # angular_vel, angle_error = self.compute_turn_vel(rel_pose, math.pi / 3, 1.0)
        # rospy.loginfo('angle_error={}'.format(angle_error))
        # is_aligned = abs(angle_error) < ANGULAR_TOL
        # cmd_vel.angular.z = angular_vel

        # Set velocity
        if self.state == 'approach':
            cmd_vel.angular.z = 0
            cmd_vel.linear.x = 0.1
        elif self.state == 'turnaround':
            cmd_vel.angular.z = -0.1
            cmd_vel.linear.x = 0

        # Print to the screen the commanded velocity
        rospy.loginfo('State: %s' % self.state)
        # rospy.loginfo('Velocity Command: %r' % cmd_vel)

        # Publish the command and save the previous output
        self.cmd_vel_pub.publish(cmd_vel)
        self.prev_vel = cmd_vel

    ####################
    # Add vision support
    def find_red(self, image):
        (rows, cols, channels) = image.shape
        cy = rows / 2
        cx = cols / 2
        px = image[cy, cx]
        rospy.loginfo('Center pixel: %r' % px)
        blue = px[0]
        green = px[1]
        red = px[2]
        return cx, cy

    def camera_callback(self, data):
        cx = 0
        cv = 0
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cx, cy = self.find_red(cv_image)
        except CvBridgeError as e:
            rospy.loginfo(e)

        # Show where the center of the red object is
        cv2.circle(cv_image, (cx, cy), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    ####################

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
