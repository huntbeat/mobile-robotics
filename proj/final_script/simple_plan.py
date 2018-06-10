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

#Same parameters we used in Lab 6
TURNING = 120
MOVING = 0.5
COLOR = 70
A_STAR = 0.01
GAUSSIAN = 10

def plan_path((sx, sy), (gx, gy)):

    nodes = {}
    nodes['START'] = (300, 700)
    nodes['A'] = (100, 100)
    nodes['B'] = (100, 200)
    nodes['C'] = (100, 300)
    nodes['D'] = (100, 400)
    nodes['E'] = (200, 400)
    nodes['F'] = (300, 400)
    nodes['G'] = (300, 500)
    nodes['H'] = (300, 600)
    nodes['I'] = (400, 400)
    nodes['J'] = (500, 400)
    nodes['K'] = (600, 400)
    nodes['L'] = (700, 400)
    nodes['M'] = (800, 400)
    nodes['N'] = (900, 400)
    nodes['O'] = (1000, 400)
    nodes['P'] = (1000, 500)
    nodes['Q'] = (1000, 600)
    nodes['R'] = (1000, 700)
    nodes['S'] = (1000, 300)
    nodes['T'] = (1000, 200)
    nodes['U'] = (1000, 100)
    nodes['V'] = (1000, 0)
    nodes['W'] = (1100, 200)
    nodes['X'] = (1200, 200)
    nodes['Y'] = (1300, 200)
    nodes['Z'] = (1400, 200)

    neighbor = {}
    neighbor['START'] = ['H','0','0','0']
    neighbor['A'] = ['0','0','B','0']
    neighbor['B'] = ['A','0','C','0']
    neighbor['C'] = ['B','0','D','0']
    neighbor['D'] = ['C','E','0','0']
    neighbor['E'] = ['0','F','0','D']
    neighbor['F'] = ['0','I','G','E']
    neighbor['G'] = ['F','0','H','0']
    neighbor['H'] = ['G','0','START','0']
    neighbor['I'] = ['0','J','0','F']
    neighbor['J'] = ['0','K','0','I']
    neighbor['K'] = ['0','L','0','J']
    neighbor['L'] = ['0','M','0','K']
    neighbor['M'] = ['0','N','0','L']
    neighbor['N'] = ['0','O','0','M']
    neighbor['O'] = ['S','0','P','N']
    neighbor['P'] = ['O','0','Q','0']
    neighbor['Q'] = ['P','0','R','0']
    neighbor['R'] = ['Q','0','0','0']
    neighbor['S'] = ['T','0','O','0']
    neighbor['T'] = ['U','W','S','0']
    neighbor['U'] = ['V','0','T','0']
    neighbor['V'] = ['0','0','U','0']
    neighbor['W'] = ['0','X','0','T']
    neighbor['X'] = ['0','Y','0','W']
    neighbor['Y'] = ['0','Z','0','X']
    neighbor['Z'] = ['0','0','0','Y']

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
		answer.insert(0,nodes[before])
                before = prev[before]
	    print answer
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

    sx,sy = (300, 700)
    gx,gy = (100, 100) #The locations of the two desks will be provided to us
    #img = 'RoadNetwork.png'
    #print('Showing %s' % img)
    #img = cv2.imread(img)
    ## Show where the center of the red object is
    pts = plan_path((sx, sy), (gx, gy))
    #cv2.circle(img, (sx, sy), 5, (0,0,255))
    #cv2.circle(img, (gx, gy), 5, (0,255,0))
    #cv2.polylines(img, np.array([pts], dtype=np.int32), False, (0,0,255))
    #cv2.imshow("My First Path", img)
    ## Timeout after 5 seconds
    #while cv2.waitKey(5) > 0: pass
