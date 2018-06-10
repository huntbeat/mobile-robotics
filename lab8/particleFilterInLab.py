from math import *
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import rospy

# landmarks which can be sensed by the robot (in meters)
#landmarks = [[20.0, 20.0]]

#landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0], [80.0, 20.0]]

#pixel_meter = 0.003048
# size of one dimension (in meters)
world_x = 1400.0 #* pixel_meter
world_y = 800.0 #* pixel_meter
start_x = 300 #* pixel_meter
start_y = 700 #* pixel_meter
straightNoise = 2#* pixel_meter
turnNoise = np.pi/72.0 #* pixel_meter
senseNoise = 2.0#* pixel_meter
PIXEL_METER = 0.00345

landmarks = [[0.0, 100.0], [100.0,600.0], [300.0, 200.0],[700.0,700.0], [900.0, 0], [1000.0, 700], [1200.0, 400.0]]
class particle:
    
    def __init__(self):
        
        sigma = 5
        self.x = sigma*np.random.randn()+ start_x        # robot's x coordinate
        self.y = sigma*np.random.randn()+ start_y          # robot's y coordinate
        self.angle = np.pi/2 +  np.random.randn() * 2.0*np.pi/36.0   # robot's angle
        self.weight = .01
	self.circle = None
	self.arrow = None

    def set(self, new_x, new_y, new_angle, weight, circle, arrow):
       
        self.x = float(new_x)
        self.y = float(new_y)
        self.angle = float(new_angle)
        self.weight = float(weight)
        self.circle = circle
	self.arrow = arrow
    def turn(self, turn_angle):
        angle = self.angle + float(turn_angle) + random.gauss(0.0, turnNoise)
        angle = self.angle % (2*pi)
        
        new = particle()
        new.set(self.x, self.y, angle)
        return new
               
    def move(self, dx, dy, dtheta):        
        # turn, and add randomness to the turning command
        angle = self.angle + float(dtheta) + random.gauss(0.0, turnNoise)
	if angle < 0:
	    angle = angle + 2.0*np.pi
	if angle > 2.0 * np.pi:
	    angle = angle - 2.0*np.pi

        # move, and add randomness to the motion command
        x = self.x + dx*np.cos(self.angle) + random.gauss(0.0, straightNoise) 
        y = self.y + -1*dx*np.sin(self.angle) + random.gauss(0.0, straightNoise)
	weight = self.weight
        # set particle
	circle = self.circle
	arrow = self.arrow
        self.set(x, y, angle, weight,circle,arrow)

    

    @staticmethod
    def gaussian(mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


    def measurement_prob(self, measurement):
        prob_dist = 0.0
        prob_angle = 0.0
        prob = 0.0
        thresh = .3
        for i in range(len(measurement)):
            measured_dist = measurement[i][1]
            measured_angle = measurement[i][0]  
            error = 100000 
            cur_dist_error = 100
	    cur_ang_error = 100
	    
	    error_idx = 1000
            for j in range(len(landmarks)):
                #rospy.loginfo(landmarks[j])
		dist_particle_to_cylinder = np.sqrt((self.x - landmarks[j][0]) ** 2 + (self.y - landmarks[j][1]) ** 2)
               
                if (landmarks[j][0] - self.x) > 0 and (landmarks[j][1] - self.y) < 0:
		    #1st Quad                
                    angle_particle = np.arctan((-1*(landmarks[j][1]-self.y))/(landmarks[j][0]-self.x))
                if (landmarks[j][0] - self.x) > 0 and (landmarks[j][1] - self.y) > 0:
                    #4th Quad
                    angle_particle = np.pi * 2 - np.arctan((landmarks[j][1]-self.y)/(landmarks[j][0]-self.x))

                if (landmarks[j][0] - self.x) < 0 and (landmarks[j][1] - self.y) < 0:
                    #2nd Quad
                    angle_particle =np.pi - np.arctan((landmarks[j][1]-self.y)/(landmarks[j][0]-self.x))


                if (landmarks[j][0] - self.x) < 0 and (landmarks[j][1] - self.y) > 0:
                    #3rd Quad
                 
                    angle_particle =np.pi + np.arctan((landmarks[j][1]-self.y)/(-1*(landmarks[j][0]-self.x)))
		#To match lidar output
                angle_particle = -1*(self.angle - angle_particle)               
                dist_error = (measured_dist-dist_particle_to_cylinder) * PIXEL_METER
                angle_error = (measured_angle-angle_particle)
		real_error = abs(dist_error)
		#rospy.loginfo('Distance dif: %s',measured_dist-dist_particle_to_cylinder)
		#rospy.loginfo('Angle dif: %s',measured_angle-angle_particle)
		#rospy.loginfo(dist_error)
		#rospy.loginfo(angle_error)
		dist_error = dist_error / measured_dist
		#if abs(dist_error*angle_error) < error:
		if abs(angle_error) < (30*np.pi*2.0/360) and real_error < error:
		    error = real_error
		    real_dist = dist_particle_to_cylinder
		    real_angle = angle_particle
                    cur_dist_error = abs(dist_error)
		    cur_ang_error = abs(angle_error)
		    error_idx = j
	    #rospy.loginfo('THIS IS THE CYLINDER IT SEES')
	    #if error_idx < 10:
            #    rospy.loginfo(landmarks[error_idx])
	    #else:
	    #	rospy.loginfo("WE Never went to if statement.")
	    #rospy.loginfo("HERE ARE THE ERRORS")
            #rospy.loginfo(cur_dist_error)
	    #rospy.loginfo(cur_ang_error)
	    prob_dist = (1/(100*cur_dist_error))**2
	    prob_angle =(1/(100*cur_ang_error))**2
        prob = prob + prob_angle *100 + prob_dist*10
        return prob   

def run((dx, dy, dtheta), cylinderInfo, particles):
    print("WE ARE RUNNING")
    n = 30
    # The robot moves the delta amounts       
    # Senses cylinder info
   
    # Update particles
    p2 = []
    for i in range(n):
        particles[i].move(dx,dy,dtheta)    
        
       
    # generate particle weights depending on robot's measurement
    w = []
    if len(cylinderInfo) != 0:
        for i in range(n):
            particles[i].weight = (particles[i].measurement_prob(cylinderInfo))
	    w.append(particles[i].weight)
    else:
	for i in range(n):
	    w.append(particles[i].weight)
    # resampling algorithm with the resampling wheel
    #First we define our new particle array
    w = np.asarray(w, dtype=np.float32)
    w = np.divide(w,w.sum())
    max_index = np.argmax(w)
    new_particles = []
    
    #new_particles = np.random.choice(particles, size=(n), replace=False, p=w)
    new_particles_indices = np.random.choice(n, size=n, p=w) 
    for i in range(n):
        #new_particles[i] = particles[new_particles_indices[i]]
        temp = particles[new_particles_indices[i]]
	x = temp.x
	y = temp.y
	angle = temp.angle
	weight = temp.weight
	new_particle = particle()
	temp.circle = None
	temp.arrow = None
	new_particle.set(x,y,angle,weight,None, None) 
	new_particles.append(new_particle)        
    particles = new_particles
    return [particles, (particles[max_index].x, particles[max_index].y, particles[max_index].angle)]
       
