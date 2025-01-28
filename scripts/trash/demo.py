import time

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('images/i00006.png', cv.IMREAD_GRAYSCALE)
start_time = time.time()
# Initiate ORB detector
orb = cv.ORB_create(nfeatures=8000)
# find the keypoints with ORB
kp = orb.detect(img, None)
# compute the descriptors with ORB
kp, des = orb.compute(img, kp)
# draw only keypoints location,not size and orientation
img2 = cv.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
print(1.0 / (time.time() - start_time))
plt.imshow(img2), plt.show()