from math import *
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# landmarks which can be sensed by the robot (in meters)
#landmarks = [[20.0, 20.0]]

#landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0], [80.0, 20.0]]

#pixel_meter = 0.003048
# size of one dimension (in meters)
world_x = 1400.0 #* pixel_meter
world_y = 800.0 #* pixel_meter
start_x = 300 #* pixel_meter
start_y = 700 #* pixel_meter
straightNoise = .1 #* pixel_meter
turnNoise = .1#* pixel_meter
senseNoise = .1#* pixel_meter

landmarks = [[200.0, 200.0], [400.0, 500.0], [900, 300]]
class particle:
    
    def __init__(self):
        
        sigma = 20
        self.x = sigma*np.random.randn()+ start_x        # robot's x coordinate
        self.y = sigma*np.random.randn()+ start_y          # robot's y coordinate
        self.angle = random.random() * 2.0 * pi   # robot's angle

        

    def set(self, new_x, new_y, new_angle):
       
        self.x = float(new_x)
        self.y = float(new_y)
        self.angle = float(new_angle)


    def sense(self):
        z = []

        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, senseNoise)
            
            angle = -1*np.arctan((self.y - landmarks[i][1])/(self.x - landmarks[i][0]))
            angle += random.gauss(0.0, senseNoise)
            
            z.append((angle,dist))

        return z

    def turn(self, turn_angle):
        angle = self.angle + float(turn_angle) + random.gauss(0.0, turnNoise)
        angle = self.angle % (2*pi)
        
        new = particle()
        new.set(self.x, self.y, angle)
        return new
               
    def forward(self, distance_travelled):
        """"
        distance_travelled = distance_travelled + random.gauss(0, straightNoise)
        angle = self.angle + random.gauss(0.0, turnNoise)
        x = self.x + cos(angle)*distance_travelled 
        x = self.x % world_x
        y = self.y + sin(angle)*distance_travelled
        y = self.y % world_y
        
        
        par = particle()
        par.set(x, y, angle)
        return par
        """
        self.move(0.0, 50.0)
    def move(self, turn, forward):
        
        # turn, and add randomness to the turning command
        angle = self.angle + float(turn) + random.gauss(0.0, turnNoise)
        angle %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, straightNoise)
        x = self.x + (cos(angle) * dist)
        y = self.y + (sin(angle) * dist)

        # cyclic truncate
        x %= world_x
        y %= world_y

        # set particle
        res = particle()
        res.set(x, y, angle)

        return res
    

    @staticmethod
    def gaussian(mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


    def measurement_prob(self, measurement):
        prob_dist = 0.0
        prob_angle = 0.0
        thresh = .5
        for i in range(len(measurement)):
            measured_dist = measurement[i][1]
            measured_angle = measurement[i][0]
            
            for j in range(len(landmarks)):
                dist_particle_to_cylinder = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
                angle_particle = np.arctan((self.y - landmarks[i][1])/(self.x - landmarks[i][0]))
                angle_particle = -1 * angle_particle #For Hunter's code to work
                dist_error = (measured_dist-dist_particle_to_cylinder)/dist_particle_to_cylinder
                angle_error = (measured_angle-angle_particle)/angle_particle
                if abs(angle_error * dist_error) < thresh:
                    #Identified a matched cylinder
                    prob_dist = prob_dist + self.gaussian(dist_particle_to_cylinder, senseNoise, measured_dist)
                    prob_angle = prob_angle + self.gaussian(angle_particle, senseNoise, measured_angle)
        prob_angle = prob_angle/len(landmarks)
        prob_dist = prob_dist/len(landmarks)
        prob = prob_angle * prob_dist
        return prob   
def visualization(robot, step, old_particles, pr, weights):


    plt.figure("Robot in the world", figsize=(15., 8.))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_x, world_y, 0]
    ax=plt.gca()
    ax.invert_yaxis()
    ax.xaxis.tick_top()                     # and move the X-Axis      
    
    ax.yaxis.tick_left()                    # remove right y-Ticks
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_x), 100)])
    plt.yticks([i for i in range(int(world_y), 0, -100)])
    
   
    rect1 = patches.Rectangle((0,100),200,400,linewidth=1, facecolor="black")
    rect2 = patches.Rectangle((200,300),200,400,linewidth=1, facecolor="black")
    rect3 = patches.Rectangle((400,300),500,200,linewidth=1, facecolor="black")
    rect4 = patches.Rectangle((900,0),200,700,linewidth=1, facecolor="black")
    rect5 = patches.Rectangle((1100,100),300,200,linewidth=1, facecolor="black")

    # Add the patch to the Axes
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(rect3)
    ax.add_patch(rect4)
    ax.add_patch(rect5)
    # draw particles
    for ind in range(len(old_particles)):

        # particle
        circle = plt.Circle((old_particles[ind].x, old_particles[ind].y), 10., facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's angle
        arrow = plt.Arrow(old_particles[ind].x, 
                          old_particles[ind].y, 
                          20*cos(old_particles[ind].angle), 
                          20*sin(old_particles[ind].angle), 
                          alpha=1., 
                          facecolor='#994c00', 
                          edgecolor='#994c00',
                          width = 5.0)
        plt.gca().add_patch(arrow)


    # draw resampled particles
    for ind in range(len(pr)):

        # particle
        circle = plt.Circle((pr[ind].x, pr[ind].y), 10., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's angle
        arrow = plt.Arrow(pr[ind].x, pr[ind].y, 
                          20*cos(pr[ind].angle), 
                          20*sin(pr[ind].angle), 
                          alpha=1., 
                          facecolor='#006600', 
                          edgecolor='#006600',
                          width=5.0)
        plt.gca().add_patch(arrow)

    # fixed landmarks of known locations
    for lm in landmarks:
        circle = plt.Circle((lm[0], lm[1]), 10., facecolor='#cc0000', edgecolor='#330000')
        plt.gca().add_patch(circle)

    # robot's location
    circle = plt.Circle((robot.x, robot.y), 10., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)

    # robot's angle
    arrow = plt.Arrow(robot.x, robot.y, 
                      20*cos(robot.angle), 
                      20*sin(robot.angle), 
                      alpha=0.5, 
                      facecolor='#000000', 
                      edgecolor='#000000',
                      width = 5.0)
    plt.gca().add_patch(arrow)

    # plt.savefig("output/figure_" + str(step) + ".png")
    plt.show()
 
def main():

    ### introduction to the robot class

    # create a robot
    myrobot = particle()
    print myrobot

    # set robot's initial position and angle
    myrobot.set(start_x, start_y, pi/2.)
    print myrobot

    # set noise parameters
    # clockwise turn and move
    
    print myrobot

    print myrobot.sense()

    print myrobot

    print myrobot.sense()

    print ''
    print ''

    ### introduction to the robot class

    # create a robot for the particle filter demo
    myrobot = particle()
    z = myrobot.sense()

    print 'z = ', z
    print 'myrobot = ', myrobot

    # create a set of particles
    n = 200 # number of particles
    p = []    # list of particles

    for i in range(n):
        x = particle()
        p.append(x)

    steps = 50  # particle filter steps

    for t in range(steps):

        # move the robot and sense the environment after that
        myrobot = myrobot.move(0, 50.0)
        z = myrobot.sense()

        p2 = []
        for i in range(n):
            
            p2.append( p[i].move(0,50.0) )
  

        p = p2
        
       
        # generate particle weights depending on robot's measurement
        w = []

        for i in range(n):
            w.append(p[i].measurement_prob(z))

        # resampling algorithm with the resampling wheel
        #First we define our new particle array
        new_particles = []

        index = int(random.random() * n)
        goalweight = 0.0
        mw = max(w)

        for i in range(n):
            goalweight += random.random() * 2.0 * mw

            #Pick the index with the greatest arc length on the wheel more often
            while goalweight > w[index]:
                
                goalweight = goalweight - w[index]
                index = (index + 1) % n

            new_particles.append(p[index])

        # here we get a set of co-located particles
        p = new_particles
        max_index = np.argmax(w)
        print 'Estimated Coordinate =', p[max_index].x, p[max_index].y, p[max_index].angle
        print 'Actuals Coordinate =', myrobot.x, myrobot.y, myrobot.angle
        print 'Step = ', t
        # visualize the current step
        visualization(myrobot, p2, new_particles, w)
    print 'p = ', p

if __name__ == "__main__":
    main()
