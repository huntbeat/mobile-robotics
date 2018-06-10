#!/usr/bin/env python
# Usage: ./simple_plan.py image.png sx sy gx gy
# ./simple_plan.py "$HOME/.gazebo/models/roadnetwork/materials/textures/RoadNetwork.png" 400 300 715 444

import cv2

from Queue import PriorityQueue

import os
import sys

import argparse

import pprint
import numpy as np

TURNING = 120
MOVING = 0.5
COLOR = 70
A_STAR = 0.01
GAUSSIAN = 10

def plan_path(img, sx, sy, gx, gy):
    nodes = {}
    nodes['A'] = (40, 270)
    nodes['B'] = (40, 360)
    nodes['C'] = (40, 180)
    nodes['D'] = (100, 270)
    nodes['E'] = (180, 270)
    nodes['F'] = (290, 270)
    nodes['H'] = (390, 270)
    nodes['I'] = (490, 270)
    nodes['J'] = (580, 270)
    nodes['K'] = (700, 270)
    nodes['L'] = (180, 30)
    nodes['M'] = (180, 90)
    nodes['N'] = (180, 180)
    nodes['O'] = (180, 350)
    nodes['P'] = (180, 430)
    nodes['Q'] = (180, 530)
    nodes['R'] = (290, 430)
    nodes['S'] = (390, 430)
    nodes['T'] = (390, 350)
    nodes['U'] = (390, 500)
    nodes['X'] = (330, 90)
    nodes['Y'] = (450, 90)
    nodes['Z'] = (650, 90)
    nodes['AA'] = (580, 180)
    nodes['BB'] = (580, 350)
    nodes['CC'] = (580, 440)
    nodes['DD'] = (580, 540)
    nodes['EE'] = (710, 440)
    nodes['FF'] = (580, 90)

    neighbor = {}
    neighbor['A'] = ['C','D','B','0']
    neighbor['B'] = ['A','0','0','0']
    neighbor['C'] = ['0','0','A','0']
    neighbor['D'] = ['0','E','0','A']
    neighbor['E'] = ['N','F','O','D']
    neighbor['F'] = ['0','H','0','E']
    neighbor['H'] = ['0','I','T','F']
    neighbor['I'] = ['0','J','0','H']
    neighbor['J'] = ['AA','K','BB','I']
    neighbor['K'] = ['0','0','0','J']
    neighbor['L'] = ['0','0','M','0']
    neighbor['M'] = ['L','X','N','0']
    neighbor['N'] = ['M','0','E','0']
    neighbor['O'] = ['E','0','P','0']
    neighbor['P'] = ['O','R','Q','0']
    neighbor['Q'] = ['P','0','0','0']
    neighbor['R'] = ['0','S','0','P']
    neighbor['S'] = ['T','0','U','R']
    neighbor['T'] = ['H','0','S','0']
    neighbor['U'] = ['S','0','0','0']
    neighbor['X'] = ['0','Y','0','M']
    neighbor['Y'] = ['0','FF','0','X']
    neighbor['Z'] = ['0','0','0','FF']
    neighbor['AA'] = ['FF','0','J','0']
    neighbor['BB'] = ['J','0','CC','0']
    neighbor['CC'] = ['BB','EE','DD', '0']                
    neighbor['DD'] = ['CC','0','0','0']
    neighbor['EE'] = ['0','0','0','CC']
    neighbor['FF'] = ['0','Z','AA','Y']                
    
    


    # Find start and end nodes based on distance from point
    ds = np.inf
    dg = np.inf
    g = {}
    for k, v in nodes.items():
        x = v[0]  
        y = v[1]

    	distance_start = (x-sx)**2 + (y-sy)**2
        distance_end = (x-gx)**2 + (y-gy)**2
        if distance_start < ds:
            ds = distance_start
            start = k
        if distance_end < dg:
            dg = distance_end
            goal = k 

    #Define Cost Function 
    	g[k] = np.inf

    print('This is the start: %s', start) 
    print('This is the goal : %s', goal ) 
    
    prev = {}
    for k,v in nodes.items():
        prev[k] = 0
 
    g[start] = 0
    q = PriorityQueue()
    direct = 1
    q.put((g[start], (start, direct)))
    while not q.empty():
        point = q.get()
        curr = point[1][0]
	y = nodes[curr][0]
	x = nodes[curr][1]
        direct = point[1][1]
	current_cost = point[0]
	if curr == goal:
	    answer = []
	    answer.append(nodes[goal])
	    before = prev[goal]
	    while not before == 0:
		answer.append(nodes[before])
                before = prev[before]
	    return answer
	else: 
	    for i in range(len(neighbor[curr])):
                neigh = neighbor[curr][i]
                if neigh != "0":
                    if (direct - i)%2 == 1: 
			turn_coefficient = 1
                    elif (direct - i) == 0:
                        turn_coefficient = 0
                    else:
                        turn_coefficient = 2
	            distance = abs((nodes[neigh][1] - nodes[curr][1]) + (nodes[neigh][0] - nodes[curr][0]))

                    new_cost = current_cost + turn_coefficient*TURNING + distance*MOVING 
		    
                    if new_cost < g[neigh]:
                         prev[neigh] = curr
                         g[neigh] = new_cost
                         q.put((g[neigh], (neigh, i)))
                   
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("img", help="Greyscale PNG image file serving as the map")
    parser.add_argument("sx", help="Starting position in x", type=int)
    parser.add_argument("sy", help="Starting position in y", type=int)
    parser.add_argument("gx", help="Goal position in x", type=int)
    parser.add_argument("gy", help="Goal position in y", type=int)
    args = parser.parse_args()

    pprint.pprint(args)

    img = args.img
    sx = args.sx
    sy = args.sy
    gx = args.gx
    gy = args.gy
    print('Showing %s' % img)
    img = cv2.imread(img)
    # Show where the center of the red object is
    pts = plan_path(img, sx, sy, gx, gy)
    cv2.circle(img, (sx, sy), 5, (0,0,255))
    cv2.circle(img, (gx, gy), 5, (0,255,0))
    cv2.polylines(img, np.array([pts], dtype=np.int32), False, (0,0,255))
    cv2.imshow("My First Path", img)
    # Timeout after 5 seconds
    while cv2.waitKey(5) > 0: pass
