from GUI import GUI
from HAL import HAL


import cv2
import numpy as np
import time

def centroid(mask):
    M = cv2.moments(mask)
    if M['m00']:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy
    else:
        raise Exception("Sorry, No red mid line found.")


def red_mask(img):
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([50, 50, 255])
    mask = cv2.inRange(img, lower_red, upper_red)
    return mask


img = HAL.getImage()
ref_x, ref_y = int(img.shape[1]/2), int(img.shape[0]/2) 
cv2.circle(img, (ref_x, ref_y), 5, (255, 255, 0), -1)
GUI.showImage(img)

kp = 0.00185
ki = 0.00001
kd = 0.00005
total_error = 0
last_error = 0
while True:
    img = HAL.getImage()
    x, y = centroid(red_mask(img))
    cv2.circle(img, (x, y), 5, (255, 255, 0), -1)
    GUI.showImage(img)
    error = ref_x - x
    P = kp * error
    total_error = total_error + error
    I = ki * total_error
    change_error = error - last_error
    D = kd * change_error
    HAL.setV(2)
    HAL.setW(P+I+D)
    last_error = error


