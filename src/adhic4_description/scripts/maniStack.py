#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


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

    lst_joint_angles_1 = [math.radians(23),
                          math.radians(70),
                          math.radians(-109),
                          math.radians(50),
                          math.radians(3),
                          math.radians(-90)]

    lst_joint_angles_2 = [math.radians(60),
                          math.radians(70),
                          math.radians(-109),
                          math.radians(50),
                          math.radians(3),
                          math.radians(-90)]

    lst_joint_angles_3 = [math.radians(-60),
                          math.radians(70),
                          math.radians(-109),
                          math.radians(50),
                          math.radians(3),
                          math.radians(-90)]

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

    while not rospy.is_shutdown():
        ur5.set_joint_angles(lst_joint_angles_1)
        
        ur5.set_joint_angles(lst_joint_angles_2)
        
        # #ur5.go_to_predefined_pose("close")          
        
        ur5.set_joint_angles(lst_joint_angles_3)
   
        # #ur5.go_to_predefined_pose("open")
      
        # ur5.set_joint_angles(lst_joint_angles_4)
      
        # ur5.set_joint_angles(lst_joint_angles_5)
      
        # ur5.set_joint_angles(lst_joint_angles_6)
      
        # ur5.go_to_predefined_pose("close")
      
        # ur5.set_joint_angles(lst_joint_angles_7)
      
        # ur5.go_to_predefined_pose("open")
      
        break
    del ur5


if __name__ == '__main__':
    main()
