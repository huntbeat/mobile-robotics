#!/usr/bin/env python
import cv2
import numpy as np

import os
import sys

def find_red(image):
    (rows, cols, channels) = image.shape
    r,g,b = cv2.split(image)
    thresh = 1
    retval, binary = cv2.threshold(g, thresh, 255, cv2.THRESH_BINARY_INV)
    loc = np.zeros(r.shape, dtype="uint8")
    loc = cv2.findNonZero(binary)
    cx = np.sum(loc[:,0,0]) / loc.shape[0]
    cy = np.sum(loc[:,0,1]) / loc.shape[0]
    print((cx,cy))
    return cx, cy

if __name__ == '__main__':

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'test_red.png'
        filename = os.path.join(
            os.getenv("HOME"), 'turtlebot_photos', filename)

    # Show the image under test
    print('Showing %s' % filename)
    img = cv2.imread(filename)
    # Show where the center of the red object is
    cx, cy = find_red(img)
    cv2.circle(img, (cx, cy), 10, 255)
    cv2.imshow("My First Red Object", img)
    # Timeout after 5 seconds
    while cv2.waitKey(5) > 0: pass
