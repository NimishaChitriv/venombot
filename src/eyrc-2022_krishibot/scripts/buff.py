#! /usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Pose
import tf

bridge = CvBridge()
global pub_rgb
global cx, cy, fx, fy


def nothing(x):
    pass


def image_processing(image):

    age=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/c_image.jpeg')
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
    # pose = []
    # global M1
    # global M2
    # global yellow
    # global red
    # cX1,cX2=0,0
    # cY1,cY2=0,0
    # global M

    # # cv2.imshow('Final1', image)
    # im_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # hsv=cv2.cvtColor(im_rgb,cv2.COLOR_BGR2HSV)
    
    # # cv2.imshow('Final2', im_rgb)
    # # # # cv2.imshow('Final', hsv)
    # # cv2.waitKey(0)
    # yelloL=(6,0,48)
    # yelloU=(179,72,121)

    # mask1=cv2.inRange(im_rgb,yelloL,yelloU)

    # cv2.imshow('Final', mask1)
    # cv2.waitKey(0)


    # kernelOpen=np.ones((5,5))
    # kernelClose=np.ones((20,20))

    # maskOpen=cv2.morphologyEx(mask1,cv2.MORPH_OPEN,kernelOpen)
    # maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)


    # contours, hierarchy = cv2.findContours(image=maskClose.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

    # image_copy = im_rgb.copy()


    # contours = sorted(contours, key=cv2.contourArea, reverse=True)



    # if(len(contours)>0):
    #     area1Max  = cv2.contourArea(contours[0])
    # count = 0

    # for c in contours:
    #     M = cv2.moments(c)
        
    #     if M["m00"] != 0:
    #         cX = int(M["m10"] / M["m00"])
    #         cY = int(M["m01"] / M["m00"])
    #     else:
    #         cX,cY=0, 0
        
    #     if(0.25*area1Max > cv2.contourArea(c)):
    #         break
    #     #print(cX,cY)
    #     count = count + 1
    #     cv2.drawContours(image_copy, [c], -1, (0, 255, 0), 2)
    #     cv2.circle(image_copy, (cX, cY), 3, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "center"+count.__str__(), (cX - 20, cY - 20),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
    #     # cv2.imshow("Image", image_copy)
    #     # cv2.waitKey(0)
    #     yellow=[cX,cY]
        
    #     pose.append(yellow)
        
    # # cv2.imshow('Final', image_copy)
    # # cv2.waitKey(0)
 
    ##########################################################################################

def main():

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    global pub_rgb
    global pub_depth
    global X1,Y1,Z1,X2,Y2
    
    rospy.init_node("percepStack", anonymous=True)
    
    rospy.sleep(10)
    
    
    image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/c_image.jpeg')
    image_processing(image1)

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

def nothing(x):
    pass   

def image_processing(image):
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
    # '''
    # NOTE: Do not modify the function name and return value.
    #       Only do the changes in the specified portion for this
    #       function.
    #       Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    # 1. Find the centroid of the bell pepper(s).
    # 2. Add the x and y values of the centroid to a list.  
    # 3. Then append this list to the pose variable.
    # 3. If multiple fruits are found then append to pose variable multiple times.

    # Input Args:
    # ------
    # image: Converted image in cv2 format.

    # Example:
    # ----
    # pose = [[x1, y1] , [x2, y2] ...... ]
    # '''
    # pose = []
    # global M1
    # global M2
    # global yellow
    # global red
    # cX1,cX2=0,0
    # cY1,cY2=0,0
    # global M
    # ############### Write Your code to find centroid of the bell peppers #####################
    # #cv2.imshow('Final1', image)
    # im_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # hsv=cv2.cvtColor(im_rgb,cv2.COLOR_BGR2HSV)
    # # cv2.imshow('Final2', im_rgb)
    # # cv2.imshow('Final', hsv)
    # # cv2.waitKey(0)
    # yelloL=(5,180,100)
    # yelloU=(20,255,255)

    # lower_red = np.array([161, 155, 84], dtype = "uint8") 
    # upper_red= np.array([179, 255, 255], dtype = "uint8")

    # mask1=cv2.inRange(hsv,yelloL,yelloU)
    # mask2=cv2.inRange(hsv,lower_red ,upper_red)

    # kernelOpen=np.ones((5,5))
    # kernelClose=np.ones((20,20))

    # maskOpen=cv2.morphologyEx(mask1,cv2.MORPH_OPEN,kernelOpen)
    # maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    # maskOpen1=cv2.morphologyEx(mask2,cv2.MORPH_OPEN,kernelOpen)
    # maskClose1=cv2.morphologyEx(maskOpen1,cv2.MORPH_CLOSE,kernelClose)

    # contours, hierarchy = cv2.findContours(image=maskClose.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # contours1, hierarchy1 = cv2.findContours(image=maskClose1.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # # draw contours on the original image
    # image_copy = image.copy()

    # # print(contours)
    # # print(len(contours1))

    # contours = sorted(contours, key=cv2.contourArea, reverse=True)
    # contours1 = sorted(contours1, key=cv2.contourArea, reverse=True)

    # # cnts1 = imutils.grab_contours(contours)
    # # cnts2 = imutils.grab_contours(contours1)

    # if(len(contours)>0):
    #     area1Max  = cv2.contourArea(contours[0])
    # if(len(contours1)>0):
    #     area2Max  = cv2.contourArea(contours1[0])

    # count = 0

    # for c in contours:
    #     M = cv2.moments(c)
        
    #     if M["m00"] != 0:
    #         cX = int(M["m10"] / M["m00"])
    #         cY = int(M["m01"] / M["m00"])
    #     else:
    #         cX,cY=0, 0
        
    #     if(0.25*area1Max > cv2.contourArea(c)):
    #         break
    #     #print(cX,cY)
    #     count = count + 1
    #     cv2.drawContours(image_copy, [c], -1, (0, 255, 0), 2)
    #     cv2.circle(image_copy, (cX, cY), 3, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "center"+count.__str__(), (cX - 20, cY - 20),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    #     # cv2.imshow("Image", image_copy)
    #     # cv2.waitKey(0)
    #     yellow=[cX,cY]
        
    #     pose.append(yellow)

    # for c in contours1:
    #     M = cv2.moments(c)
    #     if M["m00"] != 0:
    #         cX = int(M["m10"] / M["m00"])
    #         cY = int(M["m01"] / M["m00"])
            
    #     else:
    #         cX,cY=0, 0
    #     if(0.25*area2Max > cv2.contourArea(c)):
    #         break
    #     #print(cX,cY)
    #     count = count + 1
    #     cv2.drawContours(image_copy, [c], -1, (0, 255, 0), 2)
    #     cv2.circle(image_copy, (cX, cY), 3, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "center"+count.__str__(), (cX - 20, cY - 20),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    #     # cv2.imshow("Image", image_copy)
    #     # cv2.waitKey(0)
    #     red=[cX,cY]
    #     pose.append(red)
    
    #cv2.imshow('Final', image_copy)
    #cv2.waitKey(0)
    # cv2.imwrite('contours_none_image1.jpg', image_copy)
    #cv2.destroyAllWindows()
 
    ##########################################################################################
    # return pose

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
    image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/c_image.jpeg')
    image = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Final', image)
    # cv2.waitKey(0)
    image_processing(image)
    # # image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/im3.jpeg')
    # # image_processing(image1)
    # # rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
    # # rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
    # # rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
    # # # #rospy.sleep(5)

    # pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    # pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
    ####################################################################################################
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")






mport cv2
import numpy as np


def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


def getContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(area)
        if 1000 >area > 0:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt, True)
            print(peri)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            # if objCor == 3:
            #     objectType = "Tri"
            # elif objCor == 4:
            #     aspRatio = w / float(h)
            #     if aspRatio > 0.98 and aspRatio < 1.03:
            #         objectType = "Square"
            #     else:
            #         objectType = "Rectangle"
            if objCor > 4:
                objectType = "Circles"
            else:
                objectType = "None"

            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(imgContour, objectType,
                        (x + (w // 2) - 10, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)


path = "C:/Users/pushkar nerpagar/Downloads/unnamed.jpg"
img = cv2.imread(path)
imgContour = img.copy()
image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
imgBlur = cv2.GaussianBlur(image, (7, 7), 1)
ret, thresh2 = cv2.threshold(imgBlur, 75, 255, cv2.THRESH_BINARY_INV)
imgCanny = cv2.Canny(thresh2, 50, 100)
imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
#imgCanny = cv2.Canny(imgBlur, 50, 50)
getContours(imgCanny)

imgBlank = np.zeros_like(img)
imgStack = stackImages(0.8, ([img, imgGray, imgBlur],
                             [imgCanny, imgContour, imgBlank]))

img_resize=cv2.resize(imgStack, (1500, 100))
cv2.imshow("Stack", img_resize)
image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# imgBlur = cv2.GaussianBlur(image, (7, 7), 1)

# ret, thresh2 = cv2.threshold(imgBlur, 75, 255, cv2.THRESH_BINARY_INV)
#cv2.show("image_tresh",thresh2)
# imgCanny = cv2.Canny(thresh2, 50, 100)
#cv2.imshow("image",imgCanny)
getContours(imgCanny)
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

def nothing(x):
    pass   

def image_processing(image):
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
    # '''
    # NOTE: Do not modify the function name and return value.
    #       Only do the changes in the specified portion for this
    #       function.
    #       Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    # 1. Find the centroid of the bell pepper(s).
    # 2. Add the x and y values of the centroid to a list.  
    # 3. Then append this list to the pose variable.
    # 3. If multiple fruits are found then append to pose variable multiple times.

    # Input Args:
    # ------
    # image: Converted image in cv2 format.

    # Example:
    # ----
    # pose = [[x1, y1] , [x2, y2] ...... ]
    # '''
    # pose = []
    # global M1
    # global M2
    # global yellow
    # global red
    # cX1,cX2=0,0
    # cY1,cY2=0,0
    # global M
    # ############### Write Your code to find centroid of the bell peppers #####################
    # #cv2.imshow('Final1', image)
    # im_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # hsv=cv2.cvtColor(im_rgb,cv2.COLOR_BGR2HSV)
    # # cv2.imshow('Final2', im_rgb)
    # # cv2.imshow('Final', hsv)
    # # cv2.waitKey(0)
    # yelloL=(5,180,100)
    # yelloU=(20,255,255)

    # lower_red = np.array([161, 155, 84], dtype = "uint8") 
    # upper_red= np.array([179, 255, 255], dtype = "uint8")

    # mask1=cv2.inRange(hsv,yelloL,yelloU)
    # mask2=cv2.inRange(hsv,lower_red ,upper_red)

    # kernelOpen=np.ones((5,5))
    # kernelClose=np.ones((20,20))

    # maskOpen=cv2.morphologyEx(mask1,cv2.MORPH_OPEN,kernelOpen)
    # maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    # maskOpen1=cv2.morphologyEx(mask2,cv2.MORPH_OPEN,kernelOpen)
    # maskClose1=cv2.morphologyEx(maskOpen1,cv2.MORPH_CLOSE,kernelClose)

    # contours, hierarchy = cv2.findContours(image=maskClose.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # contours1, hierarchy1 = cv2.findContours(image=maskClose1.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # # draw contours on the original image
    # image_copy = image.copy()

    # # print(contours)
    # # print(len(contours1))

    # contours = sorted(contours, key=cv2.contourArea, reverse=True)
    # contours1 = sorted(contours1, key=cv2.contourArea, reverse=True)

    # # cnts1 = imutils.grab_contours(contours)
    # # cnts2 = imutils.grab_contours(contours1)

    # if(len(contours)>0):
    #     area1Max  = cv2.contourArea(contours[0])
    # if(len(contours1)>0):
    #     area2Max  = cv2.contourArea(contours1[0])

    # count = 0

    # for c in contours:
    #     M = cv2.moments(c)
        
    #     if M["m00"] != 0:
    #         cX = int(M["m10"] / M["m00"])
    #         cY = int(M["m01"] / M["m00"])
    #     else:
    #         cX,cY=0, 0
        
    #     if(0.25*area1Max > cv2.contourArea(c)):
    #         break
    #     #print(cX,cY)
    #     count = count + 1
    #     cv2.drawContours(image_copy, [c], -1, (0, 255, 0), 2)
    #     cv2.circle(image_copy, (cX, cY), 3, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "center"+count.__str__(), (cX - 20, cY - 20),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    #     # cv2.imshow("Image", image_copy)
    #     # cv2.waitKey(0)
    #     yellow=[cX,cY]
        
    #     pose.append(yellow)

    # for c in contours1:
    #     M = cv2.moments(c)
    #     if M["m00"] != 0:
    #         cX = int(M["m10"] / M["m00"])
    #         cY = int(M["m01"] / M["m00"])
            
    #     else:
    #         cX,cY=0, 0
    #     if(0.25*area2Max > cv2.contourArea(c)):
    #         break
    #     #print(cX,cY)
    #     count = count + 1
    #     cv2.drawContours(image_copy, [c], -1, (0, 255, 0), 2)
    #     cv2.circle(image_copy, (cX, cY), 3, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "center"+count.__str__(), (cX - 20, cY - 20),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    #     # cv2.imshow("Image", image_copy)
    #     # cv2.waitKey(0)
    #     red=[cX,cY]
    #     pose.append(red)
    
    #cv2.imshow('Final', image_copy)
    #cv2.waitKey(0)
    # cv2.imwrite('contours_none_image1.jpg', image_copy)
    #cv2.destroyAllWindows()
 
    ##########################################################################################
    # return pose

def getContours(imgContour,img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(area)
        if 120 >area > 100:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt, True)
            print(peri)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            
            if objCor == 3:
                objectType = "Tri"
            elif objCor == 4:
                aspRatio = w / float(h)
                if aspRatio > 0.98 and aspRatio < 1.03:
                    objectType = "Square"
                else:
                    objectType = "Rectangle"
            elif objCor > 4:
                objectType = "Circles"
            else:
                objectType = "None"

            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(imgContour, objectType,
                        (x + (w // 2) - 10, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)
    cv2.imshow('Final', imgContour)
    cv2.waitKey(0)

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
    image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/c_image.jpeg')
    image = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    
    # imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(image, (3, 3), 1)


    ret, thresh2 = cv2.threshold(imgBlur, 77, 255, cv2.THRESH_BINARY_INV)
    imgCanny = cv2.Canny(thresh2, 50, 150)
    # cv2.imshow('Final', imgCanny)
    # # getContours(image1,imgCanny)
    # cv2.waitKey(0)
    image_processing(image1)
    # # image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/im3.jpeg')
    # # image_processing(image1)
    # # rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
    # # rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
    # # rospy.sleep(3)
    # sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    # sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
    # # # #rospy.sleep(5)

    # pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    # pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
    ####################################################################################################
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")


def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver



    


# imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
# imgCanny = cv2.Canny(imgBlur, 50, 50)


# imgBlank = np.zeros_like(img)
# imgStack = stackImages(0.8, ([img, imgGray, imgBlur],
#                              [imgCanny, imgContour, imgBlank]))

# img_resize=cv2.resize(imgStack, (1500, 640))
# cv2.imshow("Stack", img_resize)

# cv2.waitKey(0)