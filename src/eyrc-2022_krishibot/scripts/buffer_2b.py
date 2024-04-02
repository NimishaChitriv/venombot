#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			percepStack.py
# Functions:		
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
# You can add more if required
##############################################################
bridge=CvBridge()

# Initialize Global variables

################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    # global pub_rgb #, add global variable if any
    global bridge

    ############################### Add your code here #######################################
    a=2
    try:
        image=bridge.imgmsg_to_cv2(img_msg,"bgr8")
    except CvBridgeError as e:
        print(e)
    ##########################################################################################
    pose = image_processing(image)
    pub_rgb.publish(str(a))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    val=[]
    ############################### Add your code here #######################################
    global pub_depth
    try:
        #print(depth_msg)
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        depth_array = np.array(depth_image, dtype=np.float32)
        bigger = cv2.resize(depth_image, (1280, 720))
        cv2.imwrite("depth_img_6.png", bigger)
        #rospy.loginfo(bigger)
        val=bigger[392][558]
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #cv2.imwrite("depth_img_2.png", depth_colormap)
        #print(depth_colormap.shape)
        #print(bigger.shape)
        
        print(val)
    except CvBridgeError as e:
        print(e)
    ##########################################################################################
    #pub_depth.publish(str(depth_val))
    

def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    #cv2.imshow("Image", hsv)
    red=[]
    yellow=[]
    yelloL=(5,180,100)
    yelloU=(20,255,255)

    # yelloL=(12,136,103)
    # yelloU=(25,255,255)
    #print(image.shape)

    # redL=(0,80,10)
    # redU=(30,300,255)
    lower_red = np.array([161, 155, 84], dtype = "uint8") 

    upper_red= np.array([179, 255, 255], dtype = "uint8")

    mask1=cv2.inRange(hsv,yelloL,yelloU)
    mask2=cv2.inRange(hsv,lower_red ,upper_red)
    # cv2.imshow("Image", mask1)
    #cv2.imshow("Image1", mask2)
    # M1 = cv2.moments(mask1)
    # M2 = cv2.moments(mask2)
    # if(M1["m00"] != 0 && M2["m00"]!=0):
    #     cX1 = int(M1["m10"] / M1["m00"])
    #     cY1 = int(M1["m01"] / M1["m00"])
    #     cX2 = int(M2["m10"] / M2["m00"])
    #     cY2 = int(M2["m01"] / M2["m00"])
    # else:
    #     cX1, cY1 = 0, 0
    #     cX2, cY2 = 0, 0
    M1 = cv2.moments(mask1)
    if M1["m00"] != 0:
        cX1 = int(M1["m10"] / M1["m00"])
        cY1 = int(M1["m01"] / M1["m00"])
        yellow=[cX1,cY1]
        pose.append(yellow)
    else:
        cX1,cY1=0, 0
    #print(cX1,cY1)
    M2 = cv2.moments(mask2)
    if M2["m00"] != 0:
        cX2 = int(M2["m10"] / M2["m00"])
        cY2 = int(M2["m01"] / M2["m00"])
        red=[cX2,cY2]
        pose.append(red)
    else:
        cX2,cY2=0, 0
        
    cv2.circle(image, (cX1, cY1), 5, (255, 255, 255), -1)
    cv2.putText(image, "centroid1", (cX1 - 25, cY1 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.circle(image, (cX2, cY2), 5, (255, 255, 255), -1)
    cv2.putText(image, "centroid2", (cX2 - 25, cY2 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    print(cX1,cY1)
    print(cX2,cY2)
    #cv2.imshow("Image", image)

    #cv2.waitKey(0)
    
    # cv2.imshow("Image window ",image)
    # cv2.waitKey(2)
    # hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    # #cv2.imshow("Image 1", hsv)
    # yelloL=(12,136,103)
    # yelloU=(25,255,255)

    # mask1=cv2.inRange(hsv,yelloL,yelloU)
    # # #mask2=cv2.inRange(hsv,redL,redU)
    # cv2.imshow("Image", mask1)
    # #cv2.imshow("Image", mask2)
    # # M1 = cv2.moments(mask1)
    # # cX1 = int(M1["m10"] / M1["m00"])
    # # cY1 = int(M1["m01"] / M1["m00"])
    # # cv2.circle(image, (cX1, cY1), 3, (255, 255, 255), -1)
    # # cv2.putText(image, "centroid1", (cX1 - 25, cY1 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # # cv2.imshow("Image", image)
    # # cv2.waitKey(0)
    ##########################################################################################
    return pose



def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    global pub_rgb
    #global pub_depth
    rospy.init_node("percepStack", anonymous=True)
    rospy.sleep(10)
    sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    #sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
    rospy.sleep(5)
    sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    #sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
    rospy.sleep(5)
    sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    #sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
    rospy.sleep(5)
    
    # 375 289
    # 663 392


    
    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")






import numpy as np
import cv2

img = cv2.imread("download2.png")
cv2.imshow("Image", img)

hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
cv2.imshow("Image", hsv)

yelloL=(5,180,100)
yelloU=(20,255,255)

redL=(30,80,10)
redU=(60,300,255)

mask1=cv2.inRange(hsv,yelloL,yelloU)
mask2=cv2.inRange(hsv,redL,redU)
cv2.imshow("Image", mask1)
#cv2.imshow("Image", mask2)
M1 = cv2.moments(mask1)
cX1 = int(M1["m10"] / M1["m00"])
cY1 = int(M1["m01"] / M1["m00"])

# M2 = cv2.moments(mask2)
# cX2 = int(M2["m10"] / M2["m00"])
# cY2 = int(M2["m01"] / M2["m00"])
 
# put text and highlight the center
cv2.circle(img, (cX1, cY1), 5, (255, 255, 255), -1)
cv2.putText(img, "centroid1", (cX1 - 25, cY1 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# cv2.circle(img, (cX2, cY2), 5, (255, 255, 255), -1)
# cv2.putText(img, "centroid2", (cX2 - 25, cY2 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

cv2.imshow("Image", img)

cv2.waitKey(0)



#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			percepStack.py
# Functions:		
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
# You can add more if required
##############################################################
bridge=CvBridge()

# Initialize Global variables

################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    global pub_rgb, bridge #, add global variable if any
    

    ############################### Add your code here #######################################
    try:
        image=bridge.imgmsg_to_cv2(img_msg,"bgr8")
    except CvBridgeError as e:
        print(e)
    ##########################################################################################
    pose = image_processing(image)
    pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    ############################### Add your code here #######################################
    global pub_depth
    ##########################################################################################
    pub_depth.publish(str(depth_val))


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################
    # cv2.imshow("Image window ",image)
    # cv2.waitKey(2)
    gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blur= cv2.GaussianBlur(gray,(5,5), cv2.BORDER_DEFAULT)
    ret, thresh = cv2.threshold(blur, 120, 200, cv2.THRESH_BINARY_INV)
    cv2.imshow("Image window 2 ",thresh)
    cv2.waitKey(2)
    ##########################################################################################
    return pose



def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    global pub_rgb
    global pub_depth
    rospy.init_node("percepStack", anonymous=True)
    sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
    
    
    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")

############################bar

# Create a window
    cv2.namedWindow('image')

    # Create trackbars for color change
    # Hue is from 0-179 for Opencv
    cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
    cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
    cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
    cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
    cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
    cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

    # Set default value for Max HSV trackbars
    cv2.setTrackbarPos('HMax', 'image', 179)
    cv2.setTrackbarPos('SMax', 'image', 255)
    cv2.setTrackbarPos('VMax', 'image', 255)

    # Initialize HSV min/max values
    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0

    while(1):
        # Get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')
        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Convert to HSV format and color threshold
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(image, image, mask=mask)

        # Print if there is a change in HSV value
        if((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
            print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax

        # Display result image
        cv2.imshow('image', result)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

def nothing(x):
    pass