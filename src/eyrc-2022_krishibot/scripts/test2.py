
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    state_description = ''
    print(regions)
    
    diff=regions['fleft']-regions['fright']
    diff_2=regions['left']-regions['right']
    print('diff==',diff)
    print('diff_2',diff_2)
    if regions['front']>1 and regions['fleft']>0.7 and regions['fright']>0.7 and abs(diff)<1:
        state_description = 'case 1 - nothing'
        velocity_msg.linear.x = 0.5
        velocity_msg.angular.z = 0.0
    elif regions['front']>1 and diff<0:
        velocity_msg.linear.x = 0.2
        velocity_msg.angular.z = -0.5
    elif regions['front']>1 and diff>0:
        velocity_msg.linear.x = 0.2
        velocity_msg.angular.z = 0.5
    elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] > 0.7 :
        state_description = 'case 2 - front'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.5
    elif regions['front'] > 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 3 - fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.5
    elif regions['front'] > 1 and regions['fleft'] < 0.7 and regions['fright'] > 0.7:
        state_description = 'case 4 - fleft'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = -0.5
    elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 5 - front and fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.5
    elif regions['front'] < 1 and regions['fleft'] < 0.7 and regions['fright'] > 0.7:
        state_description = 'case 6 - front and fleft'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = -0.5
    elif regions['front'] < 1 and regions['fleft'] < 0.7 and regions['fright'] < 0.7:
        state_description = 'case 7 - front and fleft and fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.5
    elif regions['front'] > 1 and regions['fleft'] < 0.7 and regions['fright'] < 0.7:
        state_description = 'case 8 - fleft and fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.5
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    rospy.loginfo(state_description)
    print(regions)
    pub.publish(velocity_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
        sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10) 
        velocity_msg=Twist()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass






# velocity_msg.linear.x=0.6
    # if regions['front']<1:
    #     velocity_msg.linear.x=0
    #     velocity_msg.angular.z = 0.3
    # if regions['fleft']<=regions['fright']:
    #     velocity_msg.linear.x=0.6
    #     velocity_msg.angular.z = -0.3
    # elif regions['fleft']>regions['fright']:
    #     velocity_msg.linear.x=0.6
    #     velocity_msg.angular.z = 0.3
    state_description = ''
    print(regions)
    if regions['front']>1 and regions['fleft']>0.7 and regions['fright']>0.7:
            if regions['fleft']<1.5:
                state_description = 'case 1 - nothing'
                velocity_msg.linear.x = 0.5
                velocity_msg.angular.z = 0.0
            if regions['left']>1.5:
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = 0.3
    elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] > 0.7 :
        state_description = 'case 2 - front'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    elif regions['front'] > 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 3 - fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    elif regions['front'] > 1 and regions['fleft'] < 0.7 and regions['fright'] > 0.7:
        state_description = 'case 4 - fleft'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = -1
    elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 5 - front and fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    elif regions['front'] < 1 and regions['fleft'] < 0.7 and regions['fright'] > 0.7:
        state_description = 'case 6 - front and fleft'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = -1
    elif regions['front'] < 1 and regions['fleft'] < 0.7 and regions['fright'] < 0.7:
        state_description = 'case 7 - front and fleft and fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    elif regions['front'] > 1 and regions['fleft'] < 0.7 and regions['fright'] < 0.7:
        state_description = 'case 8 - fleft and fright'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def control_loop(regions):

    # rospy.init_node('ebot_controller')
    # rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    

    # velocity_msg = Twist()
    # velocity_msg.linear.x = 0.3
    # velocity_msg.angular.z = 0
    # pub.publish(velocity_msg)
    # print("regions in c",regions['front'])
    # print('front1 in c',front1)
    print(regions['front'])
    velocity_msg.linear.x=0.3
    # if regions['front']<1:
    #     velocity_msg.linear.x=0
    #     velocity_msg.angular.z = 0.3
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():
        
        if regions['front']<1:
            velocity_msg.linear.x=0
            velocity_msg.angular.z = 0.3
        pub.publish(velocity_msg)
        # '''
        # linear_x = 0.3
        # angular_z = 0
        # print(regions)
        # velocity_msg.linear.x = linear_x 
        # velocity_msg.angular.z = angular_z
        # pub.publish(velocity_msg)
        # print("Controller message pushed at {}".format(rospy.get_time()))
        # # rate.sleep()'''