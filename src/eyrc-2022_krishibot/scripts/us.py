#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

def callback(data):
    global pub

    tbsim_vel = Twist()
    ultrasonic_value = data.data


    if( ultrasonic_value < 25 ):
        tbsim_vel.angular.z= 0.5
    else :
        tbsim_vel.linear.x=0.5

    pub.publish(tbsim_vel)
    
def listener():
    global  pub
    rospy.init_node('ultrasonic_tbsim_driver_node', anonymous=True)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("ultrasonic_values", Int16, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
    
    
#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import tf
import tf2_ros
from geometry_msgs.msg import PointStamped

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
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        rospy.Rate(100)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # rospy.loginfo(
        #     '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)
        self._planning_group = "arm"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        # rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._planning_group = "gripper"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_named_target(arg_pose_name)
        plan_success, traj, planning_time, error_code = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = traj
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        # rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    lst_joint_angles_1 = [math.radians(88),
                          math.radians(-39),
                          math.radians(-39),
                          math.radians(76),
                          math.radians(-3),
                          math.radians(0)] 
  
    # lst_joint_angles_1 = [math.radians(66),
    #                       math.radians(35),
    #                       math.radians(-92),
    #                       math.radians(27),
    #                       math.radians(-11),
    #                       math.radians(0)]

    lst_joint_angles_2 = [math.radians(86),
                          math.radians(64),
                          math.radians(-121),
                          math.radians(51),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_3 = [math.radians(-33),
                          math.radians(-5),
                          math.radians(14),
                          math.radians(25),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_4 = [math.radians(-33),
                          math.radians(-5),
                          math.radians(14),
                          math.radians(25),
                          math.radians(0),
                          math.radians(-20)]

    lst_joint_angles_5 = [math.radians(92),
                          math.radians(41),
                          math.radians(-30),
                          math.radians(0),
                          math.radians(-3),
                          math.radians(0)]

    lst_joint_angles_6 = [math.radians(92),
                          math.radians(59),
                          math.radians(-50),
                          math.radians(0),
                          math.radians(-3),
                          math.radians(0)]

    lst_joint_angles_7 = [math.radians(15),
                          math.radians(-17),
                          math.radians(13),
                          math.radians(37),
                          math.radians(0),
                          math.radians(0)]

    # lst_joint_angles_8 = [math.radians(15),
    #                       math.radians(-17),
    #                       math.radians(13),
    #                       math.radians(37),
    #                       math.radians(0),
    #                       math.radians(0)]

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()
    # tf2Broadcast = tf2_ros.TransformBroadcaster()
    # tf2Stamp = geometry_msgs.msg.TransformStamped()
    # tf2Stamp.header.stamp = rospy.Time.now()
    # tf2Stamp.header.frame_id = '/ebot_base'
    # tf2Stamp.child_frame_id = '/camera_link2'
    # tf2Stamp.transform.translation = (0.5011330950346764, 0.10998073859320247, 0.9052952116491425)
    # tf2Stamp.transform.rotation = (0.000769139475814518, 0.00044545234793776366, -0.0018722233162908417, -0.9999978523859571)
    # tf2Broadcast.sendTransform(tf2Stamp)
    # print(tf2Stamp)

    while not rospy.is_shutdown():
    
        rospy.sleep(5)
        # (trans,rot) = listener.lookupTransform('/ebot_base','/camera_link2',rospy.Time(0))
        ur5.set_joint_angles(lst_joint_angles_1)
        # print("new")
        # print(trans)
        # print(rot)
        # print("")
        # pointstamp = PointStamped()
        # pointstamp.header.frame_id = '/camera_link2'
        # pointstamp.header.stamp = rospy.Time(0)
        # pointstamp.point.x = -0.048092278646533745
        # pointstamp.point.y = -0.1957558947725106
        # pointstamp.point.z = 0.75102895
        
        # abc=listener.transformPoint('/ebot_base', pointstamp)

        # print("newwww")
        # print(pointstamp)
        # print("")
        # print(abc)
        #br.sendTransform((0.07791905597224395, 0.36513567465160474+0.29 , 1.2568037234720142),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"fruit_red", "/ebot_base")
        # ur5.set_joint_angles(lst_joint_angles_2)
        break
        """ 384352.389981, 8.554000]: position: 
        x: 0.042855510482508616
        y: -0.11054621915653334
        z: 1.2640025653588305
        orientation: 
        x: 0.03399897900341221
        y: 0.9728715244665648
        z: 0.22869453405617218
        w: 0.00799227137039542
        """
            
        """ x: 0.07793412656076547
        y: 0.36509808069519634
        z: 1.2568272261935065 


        x: 0.07791905597224395
        y: 0.36513567465160474
        z: 1.2568037234720142

        x: 0.03738559447648193
        y: 0.9981267516113556
        z: 0.04839543591861934
        w: 0.0017851682414334305
        new 
        -0.2458382086782494 0.4062525305213511 0.72301567
        -0.048092278646533745 -0.1957558947725106 0.75102895

        """

        
    del ur5


if __name__ == '__main__':
    main()
