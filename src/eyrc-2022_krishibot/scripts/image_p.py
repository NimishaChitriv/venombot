import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Pose

c_image = cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/c_image.jpeg')

im_rgb = cv2.cvtColor(c_image, cv2.COLOR_BGR2RGB)
hsv = cv2.cvtColor(im_rgb,cv2.COLOR_BGR2HSV)

cv2.imshow('Final2', im_rgb)
cv2.waitKey(0)

# lower_black = np.array([0, 41, 0], dtype = "uint8") 
# upper_black = np.array([7, 255, 255], dtype = "uint8")

# # mask1=cv2.inRange(hsv,yelloL,yelloU)
# # mask2=cv2.inRange(hsv,lower_red ,upper_red)

# kernelOpen=np.ones((5,5))
# kernelClose=np.ones((20,20))

# maskOpen=cv2.morphologyEx(mask1,cv2.MORPH_OPEN,kernelOpen)
# maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

# maskOpen1=cv2.morphologyEx(mask2,cv2.MORPH_OPEN,kernelOpen)
# maskClose1=cv2.morphologyEx(maskOpen1,cv2.MORPH_CLOSE,kernelClose)

# contours, hierarchy = cv2.findContours(image=maskClose.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
# contours1, hierarchy1 = cv2.findContours(image=maskClose1.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
