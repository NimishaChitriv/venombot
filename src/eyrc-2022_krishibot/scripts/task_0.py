#!/usr/bin/env python3

####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
##############################################################

################# ADD GLOBAL VARIABLES HERE #################
global x
global y, yaw


##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

def move1(velocity_publisher, radius):

    # declare a Twist message to send velocity commands
    velocity_message = Twist()
    # we publish the velocity at 10**10 Hz (10**10 times a second)
    loop_rate = rospy.Rate(10000)
    velocity_message.linear.x = radius
    velocity_message.angular.z = 0.99999

    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        if (y >= 7.54444):
                rospy.loginfo("reached")
                break
    
    # finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message) 

def move(velocity_publisher, speed, distance, is_forward):

    # declare a Twist message to send velocity commands
    velocity_message = Twist()
    # get current location
    x0 = x
    y0 = y

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    # we publish the velocity at 10**5 Hz (10**5 times a second)
    loop_rate = rospy.Rate(100000)

    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)


        loop_rate.sleep()

        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break
    
    # finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def rotate(velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):

    velocity_message = Twist()

    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if (current_angle_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    """ velocity_message.angular.z = 0
    while x != 2*y:
        velocity_message.linear.x = 2
        break
    print(x)
    print(y) """
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

##############################################################

def poseCallback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def main():
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        #move1(velocity_publisher, 1.0)
        rotate(velocity_publisher, 40, 90, False)#30
        #move(velocity_publisher, 1.1,2.0,True)#1.1 at battery level 12.15V

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")    

######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########    
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()
        
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")

