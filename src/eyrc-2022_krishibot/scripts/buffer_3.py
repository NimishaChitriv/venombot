
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import traceback
import sys
import math
pub = None



def clbk_laser(msg):
    

	regions = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),    
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:713]), 10),
	    }
    
	take_action(regions)
	
def take_action(regions):
	msg = Twist()
	a = rospy.get_time()
	linear_x = 0.0
	angular_z = 0.0
	state_description = ''
	#print(str(regions['left']) + " " + str(regions['fleft']) + " " + str(regions['front']) + " " + str(regions['fright']) + " " + str(regions['right']))
	if a<=18.25 and a>0.0:
	    	linear_x = 0.5
	    	angular_z = 0.0
	    	msg.linear.x = linear_x
	    	msg.angular.z = angular_z
	    	pub.publish(msg)
        if regions['front']<1:
			linear_x = 0.0
			angular_z = 0.0
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)


	elif a>26.25 and a<=36.5:
		linear_x = 0.0
		angular_z = 0.0
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		if regions['right']>0.3 and regions['right']<=0.45:
			linear_x = 0.95
			angular_z = 0.2
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
		elif regions['right']>0.45 and regions['right']<=1.0:
			linear_x = 0.95
			angular_z = -0.2
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
	elif a>36.5 and a<=39.25:
		linear_x = 0.75
		angular_z = 0.0
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		rospy.sleep(1)
		angular_speed = -math.radians(abs(90))
		linear_x = 0.0
		angular_z = angular_speed
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		rospy.sleep(1)
	elif a>39.25 and a<=43.0:
		linear_x = 0.0
		angular_z = 0.0
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		if regions['left']>0.6 and regions['left']<=1.0:
			linear_x = 0.95
			angular_z = -0.2
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
		elif regions['left']>1.0 and regions['left']<=1.25:
			linear_x = 0.95
			angular_z = 0.2
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
	elif a>43.0 and a<=44.25:
		linear_x = 0.0
		angular_z = 0.0
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		angular_speed = -math.radians(abs(90))
		linear_x = 0.0
		angular_z = angular_speed
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		rospy.sleep(1)
	elif a>44.25 and a<=54.5:
		linear_x = 0.75
		angular_z = 0.0
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
		if regions['right']>0.3 and regions['right']<=0.45:
			linear_x = 0.95
			angular_z = 0.2
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
		elif regions['right']>0.45 and regions['right']<=1.0:
			linear_x = 0.95
			angular_z = -0.2
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
	elif a>54.5 and a<=55.0:
		linear_x = 0.0
		angular_z = 0.0
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
	elif a>18.25 and a<=26.25:
		if regions['front'] <1.28 and regions['left'] <2.72 and regions['right'] < 3.07 and regions['fleft']<1.35 and regions['fright']<1.35:
			state_description = 'case 2 - left le'
			angular_speed = -math.radians(abs(60))
			linear_x = 0.0
			angular_z = angular_speed
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			linear_x = 0.25
			angular_z = 0.0
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			angular_speed = -math.radians(abs(60))
			linear_x = 0.0
			angular_z = angular_speed
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			linear_x = 0.35
			angular_z = 0.0
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			angular_speed = -math.radians(abs(45))
			linear_x = 0.0
			angular_z = angular_speed
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			linear_x = 0.5
			angular_z = 0.0
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			angular_speed = -math.radians(abs(60))
			linear_x = 0.0
			angular_z = angular_speed
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			linear_x = 0.5
			angular_z = 0.0
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1)
			angular_speed = -math.radians(abs(90))
			linear_x = 0.0
			angular_z = angular_speed
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			pub.publish(msg)
			rospy.sleep(1.95)
		rospy.loginfo(state_description)
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		pub.publish(msg)
      

def main():
    global pub
    
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()


""" hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
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
        cv2.drawContours(image=image, contours=contours1, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA) """
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
    # print(cX1,cY1)
    # # print(cX2,cY2)
    # # print(" ")
    # cv2.imshow('Final', image)
    # cv2.waitKey(27)
    # rospy.sleep(5)