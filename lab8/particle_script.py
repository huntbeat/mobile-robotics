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
import json
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import ButtonEvent

import particleFilterInLab as pf

import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray as FloatArray
from std_msgs.msg import MultiArrayLayout as ArrayLayout
from std_msgs.msg import MultiArrayDimension as ArrayDimension
from std_msgs.msg import Float64 as Float
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from matplotlib import patches

PIXEL_METER = 0.003048
PLOT_PERIOD = rospy.Duration(0.2)
world_x = 1400
world_y = 800
numPart = 30
class Controller:

    # called when an object of type Controller is created
    def __init__(self):
        rospy.init_node('particle_filter')

        rospy.loginfo('3. particle_script: E28 Final Project')

	self.cur_pose = None
	self.actual_xyd = []

        self.sensor_info = None
	
	self.t1 = 0.0
	self.t2 = 0.0

        self.particle = rospy.Subscriber('/particle', String, self.particle_callback)

        self.start_particle = rospy.Publisher('/start_particle', String,
                                queue_size=10)

	#rospy.Timer(PLOT_PERIOD, self.plot_callback)

	self.particles = []

        for i in range(numPart):
	    
            self.particles.append(pf.particle())
        
	# graph initialization
	#grid = [0, world_x, world_y, 0]
        #self.ax=plt.gca()
        #self.ax.invert_yaxis()
        #self.ax.xaxis.tick_top()        # and move the X-Axis      
        #self.ax.yaxis.tick_left()       # remove right y-Ticks
        #plt.axis(grid)
        #plt.grid(b=True, which='major', color='0.75', linestyle='--')
        #plt.xticks([i for i in range(0, int(world_x), 100)])
        #plt.yticks([i for i in range(int(world_y), 0, -100)])
        #self.start_plot = 1
	#
        #rect1 = patches.Rectangle((0,100),200,400,linewidth=1, facecolor="black")
        #rect2 = patches.Rectangle((200,300),200,400,linewidth=1, facecolor="black")
        #rect3 = patches.Rectangle((400,300),500,200,linewidth=1, facecolor="black")
        #rect4 = patches.Rectangle((900,0),200,700,linewidth=1, facecolor="black")
        #rect5 = patches.Rectangle((1100,100),300,200,linewidth=1, facecolor="black")
        #self.ax.add_patch(rect1)
        #self.ax.add_patch(rect2)
        #self.ax.add_patch(rect3)
        #self.ax.add_patch(rect4)
        #self.ax.add_patch(rect5)
        #while len(self.particles) == 0:
        #    print("zero")
	#	# graph initialization
        #for particle in self.particles:
	#    
        #    
	#    circle = plt.Circle((particle.x, particle.y), 10., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        #    self.ax.add_patch(circle)
        #    particle.circle = circle

        #    # particle's angle
        #    arrow = plt.Arrow(particle.x, particle.y, 
        #                20*np.cos(particle.angle), 
        #                -20*np.sin(particle.angle), 
        #                width=5.0)
        #    particle.arrow = arrow
        #    self.ax.add_patch(arrow)

        #plt.show() 
    def particle_callback(self, msg):
	self.t1 = time.time()
	rospy.loginfo('Received command to run pf!')
        data = msg.data
        cylinder_info = []
        rel_pose = json.loads(data)
	r_x = rel_pose.pop(0) / PIXEL_METER
	r_y = rel_pose.pop(0) / PIXEL_METER
	r_d = rel_pose.pop(0)
       	cylinder_info = rel_pose.pop(0)
	for idx in range(len(cylinder_info)):
	    cylinder_info[idx][1] = cylinder_info[idx][1] / PIXEL_METER
   	information = pf.run((r_x,r_y,r_d),cylinder_info, self.particles)
	self.particles = information[0]
	self.actual_xyd = information[1]
        packet = []
	packet.append(self.actual_xyd[0] * PIXEL_METER)
	packet.append(-1 * self.actual_xyd[1] * PIXEL_METER)
	packet.append(self.actual_xyd[2])
	rospy.loginfo("Below, is our particle coordinates:")
        rospy.loginfo(packet)
	rospy.loginfo('X and Y: %s', (self.actual_xyd[0],self.actual_xyd[1]))
	self.start_particle.publish(json.dumps(packet))
	self.t2 = time.time()
	rospy.loginfo('TOTAL TIME: %s ------------------------',self.t2-self.t1)
    
    def plot_callback(self, timer_event=None):
        if len(self.particles) == 0:
            return
		# graph initialization
        for particle in self.particles:
            if particle.circle is not None: 
	        particle.circle.remove()
            particle.circle = plt.Circle((particle.x, particle.y), 10., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
            self.ax.add_patch(particle.circle)


            # particle's angle
            if particle.arrow is not None:
	        particle.arrow.remove()
            particle.arrow = plt.Arrow(particle.x, particle.y, 
                        20*np.cos(particle.angle), 
                        -20*np.sin(particle.angle), 
                        width=5.0)
            self.ax.add_patch(particle.arrow)

        plt.draw()

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

