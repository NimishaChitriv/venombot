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

# Team ID:			kb2144
# Author List:		Vedhas Talnikar,Vishwam Talnikar,Aditya Warke,Vyankatesh
# Filename:			percepStack.py
# Functions:		
# 					[ img_clbck, depth_clbck, image_processing, main ]


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
global pub_rgb

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
    
    #global pub_rgb
    
    ############################### Add your code here #######################################
    try:
        image=bridge.imgmsg_to_cv2(img_msg,"bgr8")
    except CvBridgeError as e:
        print(e)
    ##########################################################################################
    pose = image_processing(image)
    pub_rgb.publish(str(pose))
    #print(pose)

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
    global yellow
    global red
    
    
    ############################### Add your code here #######################################
    global pub_depth
    try:
        #print(depth_msg)
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        depth_array = np.array(depth_image, dtype=np.float32)
        bigger = cv2.resize(depth_image, (1280, 720))
        #cv2.imwrite("depth_img_6.png", bigger)
        #rospy.loginfo(bigger)
        if M1["m00"] != 0:
            val=bigger[yellow[1]][yellow[0]] 
            depth_val.append(round(val*0.001,1))        
        else:
            yellow=[]

        if M2["m00"] != 0:
            val2=bigger[red[1]][red[0]]
            depth_val.append(round(val2*0.001,1))
        else:
            red=[]
        # val=bigger[yellow[1]][yellow[0]]
        # val2=bigger[red[1]][red[0]]
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #cv2.imwrite("depth_img_2.png", depth_colormap)
        #print(depth_colormap.shape)
        #print(bigger.shape)
        # depth_val.append(val)
        # depth_val.append(val2)
        # print(val)
        # print(val2)
        # print(" ")
        #print(depth_val)
    except CvBridgeError as e:
        print(e)
    # ##########################################################################################
    pub_depth.publish(str(depth_val))
    depth_val = []
    

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
    global M1
    global M2
    global yellow
    global red
    cX1,cX2=0,0
    cY1,cY2=0,0
    ############### Write Your code to find centroid of the bell peppers #####################

    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    yelloL=(14,144,0)
    yelloU=(179,255,255)

    kernelOpen=np.ones((5,5))
    kernelClose=np.ones((20,20))

    lower_red = np.array([161, 155, 84], dtype = "uint8")
    upper_red= np.array([179, 255, 255], dtype = "uint8")

    mask1=cv2.inRange(hsv,yelloL,yelloU)
    mask2=cv2.inRange(hsv,lower_red ,upper_red)

    maskOpen=cv2.morphologyEx(mask1,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose

    conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

    

    cv2.imshow('Final1', maskFinal)
    #contours, hierarchy = cv2.findContours(image=mask1, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    for c in range(len(conts)):
        M1 = cv2.moments(maskFinal) 
        if M1["m00"] != 0:
            cX1 = int(M1["m10"] / M1["m00"])
            cY1 = int(M1["m01"] / M1["m00"])
            yellow=[cX1,cY1]
            pose.append(yellow)
        else:
            cX1,cY1=0,0
        #print(cX1,cY1)
        cv2.circle(image, (cX1, cY1), 3, (255, 255, 255), -1)
        cv2.putText(image, "center", (cX1 - 25, cY1 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.drawContours(image=image, contours=conts, contourIdx=-1, color=(10, 255, 0), thickness=1, lineType=cv2.LINE_AA)
    contours1, hierarchy1 = cv2.findContours(image=mask2, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    for c in contours1:
        M2 = cv2.moments(mask2)
        if M2["m00"] != 0:
            cX2 = int(M2["m10"] / M2["m00"])
            cY2 = int(M2["m01"] / M2["m00"])
            red=[cX2,cY2]
            pose.append(red)
        else:
            cX2,cY2=0,0
        cv2.circle(image, (cX2, cY2), 3, (255, 255, 255), -1)
        cv2.putText(image, "center2", (cX2 - 25, cY2 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.drawContours(image=image, contours=contours1, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)
    # M1 = cv2.moments(mask1)
    # if M1["m00"] != 0:
    #     cX1 = int(M1["m10"] / M1["m00"])
    #     cY1 = int(M1["m01"] / M1["m00"])
    #     yellow=[cX1,cY1]
    #     pose.append(yellow)
    # else:
    #     cX1,cY1=0,0
    # #print(cX1,cY1)
    # M2 = cv2.moments(mask2)
    # if M2["m00"] != 0:
    #     cX2 = int(M2["m10"] / M2["m00"])
    #     cY2 = int(M2["m01"] / M2["m00"])
    #     red=[cX2,cY2]
    #     pose.append(red)
    # else:
    #     cX2,cY2=0,0

    # contours, hierarchy = cv2.findContours(image=mask1, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # contours1, hierarchy1 = cv2.findContours(image=mask2, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # # draw contours on the original image
    # image_copy = image.copy()
    # cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(10, 255, 0), thickness=1, lineType=cv2.LINE_AA)
    # cv2.drawContours(image=image_copy, contours=contours1, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)

    # cv2.circle(image_copy, (cX1, cY1), 3, (255, 255, 255), -1)
    # cv2.putText(image_copy, "center1", (cX1 - 25, cY1 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # cv2.circle(image_copy, (cX2, cY2), 3, (255, 255, 255), -1)
    # cv2.putText(image_copy, "center2", (cX2 - 25, cY2 - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    # # see the results
    print(cX1,cY1)
    # print(cX2,cY2)
    # print(" ")
    cv2.imshow('Final', image)
    cv2.waitKey(27)
    rospy.sleep(5)
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
    image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/img5.jpg')
    image_processing(image1)

    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
    # #rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
    # #rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
    # # #rospy.sleep(5)

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




#! /usr/bin/env python3
# #rospy.sleep(5)
#     image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/im1.jpeg')
#     image2=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/im2.jpeg')
#     image3=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/im3.jpeg')
#     #cv2.imshow('im1',image1)
#     #cv2.waitKey(0)
#     image_processing(image1)
#     image_processing(image2)
#     image_processing(image3)
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

# Team ID:			kb2144
# Author List:		Vedhas Talnikar,Vishwam Talnikar,Aditya Warke,Vyankatesh
# Filename:			percepStack.py
# Functions:		
# 					[ img_clbck, depth_clbck, image_processing, main ]


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
global pub_rgb
global M1

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
    
    #global pub_rgb
    
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
    val=[]
    global yellow
    global red
    global M1
    global M2
    
    ############################### Add your code here #######################################
    #global pub_depth
    try:
        #print(depth_msg)
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        depth_array = np.array(depth_image, dtype=np.float32)
        bigger = cv2.resize(depth_image, (1280, 720))
        #cv2.imwrite("depth_img_6.png", bigger)
        #rospy.loginfo(bigger)
        if M3["m00"] != 0:
            val=bigger[yellow[1]][yellow[0]] 
            depth_val.append(val)        
        else:
            yellow=[]

        if M4["m00"] != 0:
            val2=bigger[red[1]][red[0]]
            depth_val.append(val2)
        else:
            red=[]
        # val=bigger[yellow[1]][yellow[0]]
        # val2=bigger[red[1]][red[0]]
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #cv2.imwrite("depth_img_2.png", depth_colormap)
        #print(depth_colormap.shape)
        #print(bigger.shape)
        # depth_val.append(val)
        # depth_val.append(val2)
        # print(val)
        # print(val2)
        # print(" ")
    except CvBridgeError as e:
        print(e)
    # ##########################################################################################
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
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    global yellow
    global red
    global M1
    global M2
    global M3,M4
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
    M3=M1
    if M1["m00"] != 0:
        cX1 = int(M1["m10"] / M1["m00"])
        cY1 = int(M1["m01"] / M1["m00"])
        yellow=[cX1,cY1]
        pose.append(yellow)
    else:
        cX1,cY1=0, 0

    #print(cX1,cY1)
    M2 = cv2.moments(mask2)
    M4=M2
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
    # print(cX1,cY1)
    # print(cX2,cY2)
    cv2.imshow("Image", image)

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
    global pub_depth
    
    rospy.init_node("percepStack", anonymous=True)
    #rospy.sleep(5)
    sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    #sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
    # #rospy.sleep()
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
    # #rospy.sleep(5)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
    # #rospy.sleep(5)

    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    #pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
    ####################################################################################################
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")