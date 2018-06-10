# ENGR028 Homework 2 - Hyong Hark Lee - 09/22/17

import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv 

T = 5.0

def tfmat(theta,pair):
  a = np.sin(theta)
  b = np.cos(theta)
  t1, t2 = pair

  return np.matrix([[a, -b, t1], [b, a, t2], [0, 0, 1]])

def WtoB(time):
  theta = 7 * np.pi / 6
  x = 3.5 + 0.25 * np.cos(theta) * time
  y = 2.0 + 0.25 * np.sin(theta) * time
  return tfmat(theta, (x,y))

def BtoC(time):
  theta = 0
  x = 0.25
  y = 0
  return tfmat(theta, (x,y))

def WtoA(time):
  theta = np.pi * -1 / 10 * time
  x = 0.5 + 2 * np.sin(np.pi * time / 10)
  y = 2 * np.cos(np.pi * time / 10)
  return tfmat(theta, (x,y))

def AtoL(time):
  theta = 0
  x = 0.25
  y = 0
  return tfmat(theta, (x,y))

# setting C as the origin, we must go from C to B to W to A to L

# we begin from C

answer = []

for time in np.arange(0.0, T, 0.1):
  C = np.matrix('0;0;1')
  B = np.dot(inv(BtoC(time)), C)
  W = np.dot(inv(WtoB(time)), B)
  A = np.dot(WtoA(time), W)
  L = np.dot(AtoL(time), A)
  answer.append((L[0],L[1]))

answer = np.array(answer)

plt.plot(np.arange(0.0, T, 0.1), answer[:,0], 'b')
plt.plot(np.arange(0.0, T, 0.1), answer[:,1], 'r')
plt.show()

