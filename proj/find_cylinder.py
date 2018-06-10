#!/usr/bin/env python
import cv2
import numpy as np

import os
import sys

def find_cylinder(image):
    (rows, cols, channels) = image.shape
    laser_row = rows/2
    crop_top = int(rows*2.2/5)
    crop_bottom = rows #int(rows*4.0/5)
    laser_row = laser_row - crop_top
    image = image[crop_top:crop_bottom,:,:]
    cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(image)
    mask_s = cv2.inRange(s,40,150)
    mask_h = cv2.inRange(h,40,150)
    binary = np.bitwise_and(mask_s, mask_h)
    return h
    return binary
    kernel = np.ones((3,3),np.uint8)    
    clean = cv2.morphologyEx(binary,cv2.MORPH_OPEN,kernel)
    return close

def find_red(image):
    (rows, cols, channels) = image.shape
    r,g,b = cv2.split(image)
    print image.shape
    thresh = 1
    retval, binary = cv2.threshold(g, thresh, 255, cv2.THRESH_BINARY_INV)

if __name__ == '__main__':

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'test_red.png'
        filename = os.path.join(
            os.getenv("HOME"), 'turtlebot_photos', filename)

    # Show the image under test
    img = cv2.imread(filename)
    # Show where the center of the red object is
    img = find_cylinder(img)
    cv2.imshow("Image window", img)
    while cv2.waitKey(3) > 0: pass;
