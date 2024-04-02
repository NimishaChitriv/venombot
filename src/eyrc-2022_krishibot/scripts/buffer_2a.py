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

    lst_joint_angles_1 = [math.radians(66),
                          math.radians(47),
                          math.radians(-92),
                          math.radians(27),
                          math.radians(-11),
                          math.radians(0)]

    lst_joint_angles_2 = [math.radians(68),
                          math.radians(70),
                          math.radians(-119),
                          math.radians(29),
                          math.radians(-11),
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

    while not rospy.is_shutdown():
        ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(0)
        ur5.set_joint_angles(lst_joint_angles_2)
        rospy.sleep(0)
        ur5.go_to_predefined_pose("close")
        rospy.sleep(0)
        ur5.set_joint_angles(lst_joint_angles_3)
        rospy.sleep(0)
        ur5.go_to_predefined_pose("open")
        rospy.sleep(0)
        ur5.set_joint_angles(lst_joint_angles_4)
        rospy.sleep(0)
        ur5.set_joint_angles(lst_joint_angles_5)
        rospy.sleep(0)
        ur5.set_joint_angles(lst_joint_angles_6)
        rospy.sleep(0)
        ur5.go_to_predefined_pose("close")
        rospy.sleep(0)
        ur5.set_joint_angles(lst_joint_angles_7)
        rospy.sleep(0)
        ur5.go_to_predefined_pose("open")
        rospy.sleep(0)
        break
    del ur5


if __name__ == '__main__':
    main()


#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg2_predefined_pose', anonymous=True)

        self._planning_group = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

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

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan_success, traj, planning_time, error_code = self._group.plan()
        #plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = traj
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    while not rospy.is_shutdown():
        ur5.go_to_predefined_pose("close")
        rospy.sleep(2)
        ur5.go_to_predefined_pose("open")
        rospy.sleep(2)

    del ur5


if __name__ == '__main__':
    main()




import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math 
from tf.transformations import quaternion_from_euler

def go_to_pose(arg_pose):
    commander = moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    planning_group = "arm"
    group = moveit_commander.MoveGroupCommander(planning_group)
    pose_values = group.get_current_pose().pose
    rospy.loginfo(pose_values)
    group.set_pose_target(arg_pose)
    flag_plan = group.go(wait=True)

    if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
    else:
        rospy.logerr(
            '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

    return flag_plan

def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():

    q=quaternion_from_euler(0.00, 0.100000, -3.140001)

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.0400000
    ur5_pose_3.position.y = -0.1170000
    ur5_pose_3.position.z = 1.020000
    ur5_pose_3.orientation.x = q[0]
    ur5_pose_3.orientation.y = q[1]
    ur5_pose_3.orientation.z = q[2]
    ur5_pose_3.orientation.w = q[3]

    while not rospy.is_shutdown():
        go_to_pose(ur5_pose_3)
        rospy.sleep(2)

    del ur5

#     x: 0.039981926230448175
#   y: -0.11704618994190569
#   z: 1.0200692783339027
# orientation: 
#   x: 0.04975787824454915
#   y: 0.0004663302148415819
#   z: -0.998761021353817
#   w: 0.000598592525585417


if __name__ == '__main__':
    main()



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

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.0400000
    ur5_pose_3.position.y = -0.1170000
    ur5_pose_3.position.z = 1.020000
    ur5_pose_3.orientation.x = 0.635613875737
    ur5_pose_3.orientation.y = 0.77190802743
    ur5_pose_3.orientation.z = 0.00233308772292
    ur5_pose_3.orientation.w = 0.0121472162087

    

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


#! /usr/bin/env python3

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

        self._planning_group = "ur5_1_planning_group"
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

