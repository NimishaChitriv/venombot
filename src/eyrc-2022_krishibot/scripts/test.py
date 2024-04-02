#18 oct 20:19
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
        velocity_msg.linear.x = 0.7
        velocity_msg.angular.z = 0.0
        # if 1<abs(diff)<1.5:
        #     velocity_msg.linear.x = 0.2
        #     velocity_msg.angular.z = 0.5
        # if 1<abs(diff)<1.5:
        #     velocity_msg.linear.x = 0.2
        #     velocity_msg.angular.z = 0.5
        # if regions['fleft']>=2 and regions['right']>2 and regions['left']>2:
        #     velocity_msg.angular.z =0.5
        #if diff<0:
            #velocity_msg.linear.x = 0.2
            #velocity_msg.angular.z =0.5
        # if diff_2<0:
        #     velocity_msg.angular.z =0.5
        
    elif regions['front']>1 and diff<0:
        velocity_msg.linear.x = 0.3
        velocity_msg.angular.z = -0.5
    elif regions['front']>1 and diff>0:
        velocity_msg.linear.x = 0.3
        velocity_msg.angular.z = 0.5
    elif regions['front'] < 1 :
        state_description = 'case 2 - front'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.5
    if regions['front']>2 and regions['fright']>1.7 and regions['right']>2:
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0





def control_loop():
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():
        
        linear_x = 0
        angular_z = 0

        state_description = ''

        if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 2 - front'
            linear_x = 0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state_description = 'case 3 - fright'
            linear_x = 0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state_description = 'case 4 - fleft'
            linear_x = 0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state_description = 'case 5 - front and fright'
            linear_x = 0
            angular_z = -0.3
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state_description = 'case 6 - front and fleft'
            linear_x = 0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 7 - front and fleft and fright'
            linear_x = 0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0
            angular_z = -0.3
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)

        rospy.loginfo(state_description)
        
        velocity_msg.linear.x = linear_x
        velocity_msg.angular.z = angular_z
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass


 # global front1
    # front1=min(min(msg.ranges[288:431]), 10)
    # if msg.ranges[0] > 1:
    #     velocity_msg.linear.x = 0.5
    #     velocity_msg.angular.z = 0.0
    # print("Number of ranges: ", len(msg.ranges))
    # print("Reading at position 0:", msg.ranges[0])
    # if msg.ranges[0] < 1:
    #     velocity_msg.linear.x = 0.0
    #     velocity_msg.angular.z = 0.0
    print("regions",regions['front'])
    print("regions",regions['fleft'])
    print("regions",regions['fright'])
    # print('front1',front1)
    # print('direct',min(min(msg.ranges[288:431]), 10))
    # print('360=',msg.ranges[360])
    velocity_msg.linear.x=0.3
    if regions['front']<1:
        velocity_msg.linear.x=0
        velocity_msg.angular.z = 0.3
    pub.publish(velocity_msg)
    # control_loop(regions)
    if regions['front'] > 1 and regions['fleft'] > 0.5 and regions['fright'] > 0.5:
        velocity_msg.linear.x = 0.3
        velocity_msg.angular.z = 0.0




# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# def callback(msg):
#     print(msg.ranges[360])
#     move.linear.x=0.3
#     if msg.ranges[360]<1:
#         move.linear.x=0
#     pub.publish(move)

# rospy.init_node('ebot_controller')
# sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, callback)
# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# move=Twist()
# rospy.spin()


# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import LaserScan

# def callback(msg):
#     print('values at 0 degree')
#     print(msg.ranges[0])
#     print('values at 90 degree')
#     print( msg.ranges[360])
#     print('values at 180 degree')
#     print(msg.ranges[719])

# rospy.init_node('ebot_controller')
# sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, callback)
# rospy.spin()

# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# rospy.init_node('ebot_controller')

# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# rate=rospy.rate(1)
# rot=Twist()
# rot.angular.z=0.5

# while not rospy.is_shutdown():
#     pub.publish(rot)
#     rate.sleep()