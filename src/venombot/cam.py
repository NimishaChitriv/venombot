#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from venombot.msg import Tracker  # Assuming the Tracker message type exists

class RedRectangleDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera1/color/image_raw1', Image, self.image_callback)
        self.centroid_pub = rospy.Publisher('/cxy', Tracker, queue_size=10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define red color range in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours to find the rectangle
        for contour in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it's likely a rectangle
            if len(approx) == 4:
                # Draw the rectangle on the original image
                cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                # Find the centroid of the rectangle
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(image, f"Centroid: ({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    # Publish centroid on /cxy topic
                    centroid_msg = Tracker()
                    centroid_msg.x = cx
                    centroid_msg.y = cy
                    centroid_msg.flag1 = True  # Assuming the flag is set to true when centroid is detected
                    self.centroid_pub.publish(centroid_msg)

        # Display the image
        cv2.imshow("Red Rectangle Detection", image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("red_rectangle_detector", anonymous=False)
    detector = RedRectangleDetector()
    rospy.spin()

