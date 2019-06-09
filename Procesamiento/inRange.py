import numpy as np
import cv2

green = np.uint8([[[0,255,0]]])
hsv_green =  cv2.cvtColor(green,cv2.COLOR_RGB2HSV)
print(hsv_green)
