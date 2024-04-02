"""
import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg1_node_print_pose_joint_angles', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "roll: {}\n".format(roll) +
                      "pitch: {}\n".format(pitch) +
                      "yaw: {}\n".format(yaw) +
                      '\033[0m')

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    while not rospy.is_shutdown():
        ur5.print_pose_ee()
        ur5.print_joint_angles()
        rospy.sleep(1)

    del ur5


if __name__ == '__main__':
    main()
     """
""" 
[INFO] [1667565690.408872, 293.006000]: 
End-Effector (wrist_3_link) Pose: 

x: 0.12531363578868498
y: 0.4182557588377989
z: 1.1850049536704401

roll: 2.712795687346876
pitch: 0.06059499368162933
yaw: 2.798051432168354

[INFO] [1667565690.444331, 293.028000]: 
Joint Values: 

ur5_shoulder_pan_joint: 79.66641730373141
ur5_shoulder_lift_joint: 51.42509696869717
ur5_elbow_joint: -87.23267000370286
ur5_wrist_1_joint: 11.087567954722463
ur5_wrist_2_joint: 7.060090422968602
ur5_wrist_3_joint: -7.055071169089273

End-Effector (wrist_3_link) Pose: 

x: 0.20319818310177556
y: 0.4288092682607799
z: 1.1937959549221313

roll: 2.890529234127998
pitch: -0.07070850718412783
yaw: 3.0030861782388096

[INFO] [1667566085.283606, 492.302000]: 
Joint Values: 

ur5_shoulder_pan_joint: 69.57692035025843
ur5_shoulder_lift_joint: 57.47573661858707
ur5_elbow_joint: -97.30836750328963
ur5_wrist_1_joint: 25.205542619561864
ur5_wrist_2_joint: -11.08743901612994
ur5_wrist_3_joint: 7.055188027195294


 """



""" 
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import quaternion_from_euler


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    # ur5_pose_1 = geometry_msgs.msg.Pose()
    # ur5_pose_1.position.x = -0.817261772949
    # ur5_pose_1.position.y = -0.109110076352
    # ur5_pose_1.position.z = 0.94446979642
    # ur5_pose_1.orientation.x = -0.999999995957
    # ur5_pose_1.orientation.y = 4.37354574363e-05
    # ur5_pose_1.orientation.z = 7.85715579538e-05
    # ur5_pose_1.orientation.w = 2.12177767514e-09

    # ur5_pose_2 = geometry_msgs.msg.Pose()
    # ur5_pose_2.position.x = -0.414925357653
    # ur5_pose_2.position.y = 0.284932768677
    # ur5_pose_2.position.z = 1.78027849967
    # ur5_pose_2.orientation.x = -0.199396929724
    # ur5_pose_2.orientation.y = 1.64394297608e-05
    # ur5_pose_2.orientation.z = 0.979918803013
    # ur5_pose_2.orientation.w = 6.03911583936e-05
    roll= 2.890529234127998
    pitch= -0.07070850718412783
    yaw= 3.0030861782388096
    q=quaternion_from_euler(roll, pitch, yaw)
    # x: -0.07267658266328164
    # y: -0.9889116566195059
    # z: -0.12684981802586684
    # w: 0.026095467063383736 


    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.20319818310177556
    ur5_pose_3.position.y = 0.4288092682607799
    ur5_pose_3.position.z = 1.1937959549221313
    ur5_pose_3.orientation.x = -0.07248638061283189
    ur5_pose_3.orientation.y = -0.988938524562905
    ur5_pose_3.orientation.z = -0.1266482420661716
    ur5_pose_3.orientation.w = 0.0265808585742584

    #     ller failed during execution 0.04 -0.117 1.02 0.0 0.1 -3.14
    # [INFO] [1667127008.265798, 32.469000]: >>> Final Pose:
    # [INFO] [1667127008.272303, 32.472000]: position: 
    #   x: 0.03993721632235461
    #   y: -0.1170437768730037
    #   z: 1.0200455703739582
    # orientation: 
    #   x: 0.050067173074984324
    #   y: 0.00038499276437969524
    #   z: -0.9987457058264966
    #   w: 0.0003808595334731235
    # [INFO] [1667127008.304971, 32.479000]: >>> Final Joint Values:
    # [INFO] [1667127008.311729, 32.494000]: [-3.07778624497295, 0.11150692197279355, -0.04555156429813323, 3.087247057061251, -1.633807773277434, 0.08859245880743849]


    while not rospy.is_shutdown():
        # ur5.go_to_pose(ur5_pose_1)
        # rospy.sleep(2)
        # ur5.go_to_pose(ur5_pose_2)
        # rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2)

    del ur5


if __name__ == '__main__':
    main()
 """



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
    velocity_msg.linear.x = 0.5
    
    diff= regions['fleft']-regions['fright']
    t0 = rospy.Time.now().to_sec()

    if t0<=6.4:
        if regions['front']>=1 and regions['fleft']>0.7 and regions['fright']>0.7 and abs(diff)<1:
            if  diff<0:
                state_description = 'case 1 - nothing and diff<0 '
                velocity_msg.linear.x = 1.5
                velocity_msg.angular.z = 10*diff
            elif diff>0:
                state_description = 'case 2 - nothing and diff>0 '
                velocity_msg.linear.x = 1.5
                velocity_msg.angular.z = 10*diff
        
        elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] > 0.7:
            state_description = 'case 2 - front'
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = 0
        
        rospy.loginfo(state_description)
        
    elif t0>6.4 and t0<=11 or t0>17.6 and t0<=20 or t0>21.2 and t0<=23.6:
        state_description = 'Initiating the semi-circle'
        velocity_msg.linear.x = 0.5
        velocity_msg.angular.z = 1.2

        rospy.loginfo(state_description)
    elif t0>11 and t0<=17.8 or t0>23.6 and t0<=30.8:
        
        if regions['front']>=0.7 and regions['fleft']>0 and regions['right']>0.7:
            if  regions['left']<0.5:
                state_description = 'case 1 - nothing  '
                velocity_msg.linear.x = 1.2
                velocity_msg.angular.z = -0.7
            elif regions['left']>0.5:
                state_description = 'case 2 - nothing  '
                velocity_msg.linear.x = 1.2
                velocity_msg.angular.z = 0.7

        elif regions['front'] < 0.7 and regions['fleft'] > 0 and regions['fright'] > 0.7:
            state_description = 'case 2 - front'
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = -0.5

        rospy.loginfo(state_description)
        

    elif t0>20 and t0<21.2:
        
        if regions['front']>=1 and regions['fleft']>0:
            if  regions['right']>1.0:
                state_description = 'case 1 - nothing '
                velocity_msg.linear.x = 1.2
                velocity_msg.angular.z = -1
            elif regions['right']<1.0:
                state_description = 'case 2 - nothing '
                velocity_msg.linear.x = 1.2
                velocity_msg.angular.z = 1
            
        elif regions['front'] < 1 and regions['fleft'] > 0 and regions['fright'] > 0.7:
            state_description = 'case 2 - front'
            velocity_msg.linear.x = 0.2
            velocity_msg.angular.z = -0.2
        print(regions)
        rospy.loginfo(state_description)
        print('diff==',diff)
        print('time==',t0)

    else:
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0

    
    pub.publish(velocity_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
        sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(100) 
        velocity_msg=Twist()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
global ka, kp
kp=2.5
ka=5

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
    # velocity_msg.linear.x = 1
    # pub.publish(velocity_msg)
    # rospy.sleep(1)
    print('diff==',diff)
    print('diff_2',diff_2)
    if regions['front']>0.8 and regions['fleft']>0.7 and regions['fright']>0.7  and abs(diff)<1:
        state_description = 'case 1 - nothing'
        velocity_msg.linear.x = 0.7
        velocity_msg.angular.z = 0.0
    
        #pub.publish(velocity_msg)
        # if regions['fright']>1.5 and regions['fleft']<1:
        #     state_description = 'case 1 - nothing.2'
        #     velocity_msg.linear.x = 0
        #     velocity_msg.angular.z = 0.5

        # if 1<abs(diff)<1.5:
        #     velocity_msg.linear.x = 0.2
        #     velocity_msg.angular.z = 0.5
        # if 1<abs(diff)<1.5:
        #     velocity_msg.linear.x = 0
        #     velocity_msg.angular.z = 0.5
        # if regions['fleft']>=2 and regions['right']>2 and regions['left']>2:
        #     velocity_msg.angular.z =0.5
        # if diff<0:
        #     #velocity_msg.linear.x = 0
        #     velocity_msg.angular.z =0.5
        # if diff_2<0:
        #     velocity_msg.angular.z =0.5
    elif regions['front'] < 0.8 and regions['fleft'] > 0.7 and regions['fright'] > 0.7:
        state_description = 'case 2 - front'
        velocity_msg.linear.x = 0.1*regions['left']
        velocity_msg.angular.z = 0.5*regions['left']
    elif regions['front']>0.8 and regions['fright']>1.5:
        velocity_msg.angular.z=0.5
    elif regions['front'] > 1 and  diff<0:
        velocity_msg.linear.x = 0.8*diff
        velocity_msg.angular.z = 5*diff
    elif regions['front'] > 1 and diff>0:
        velocity_msg.linear.x = 0.8*diff
        velocity_msg.angular.z = 5*diff
    
    # if regions['front']>2 and regions['fright']>1.7 and regions['right']>2:
    #     velocity_msg.linear.x = 0
    #     velocity_msg.angular.z = 0

   
    # else:
    #     state_description = 'unknown case'
    #     rospy.loginfo(regions)
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
    
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    linear_x = 0.5
    angular_z = 0

    state_description = ''

    if regions['front'] > 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 1 - nothing'
        linear_x = 0.5
        angular_z = 0
    
    elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] > 0.7:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.8
    elif regions['front'] > 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.9
    elif regions['front'] > 1 and regions['fleft'] < 0.7 and regions['fright'] > 0.7:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['fleft'] > 0.7 and regions['fright'] < 0.7:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < 1 and regions['fleft'] < 0.7 and regions['fright'] > 0.7:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['fleft'] < 0.7 and regions['fright'] < 0.7:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.8
    elif regions['front'] > 1 and regions['fleft'] < 0.7 and regions['fright'] < 0.7:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['fleft'] > 0.7 and regions['fright'] > 0.7 and regions['fright']<1.5:
        state_description = 'case 1 - nothing.2'
        linear_x = 0.9
        angular_z = 0.3
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
        
    velocity_msg.linear.x = linear_x
    velocity_msg.angular.z = angular_z
    pub.publish(velocity_msg)
