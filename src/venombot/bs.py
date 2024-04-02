#!/usr/bin/env python3

import rospy
import random
import os
from std_msgs.msg import Int8MultiArray
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench, SetModelState
from geometry_msgs.msg import Pose, Wrench, Vector3, Twist
from gazebo_msgs.msg import ModelState

stopped_blocks = []

# Function to convert integer to string
def int_to_string(a):
    return str(a)

def model_states_callback(msg):
    # This dictionary will hold the latest pose for each block
    block_poses = {}

    # Iterate through all models and update poses for red_blocks_
    for i, model_name in enumerate(msg.name):
        if model_name.startswith("red_blocks_"):
            block_poses[model_name] = msg.pose[i]

    # For each block, check if it needs to be stopped and act accordingly
    for model_name, pose in block_poses.items():
        if model_name not in stopped_blocks:  # Check if the block is not already stopped
            block_x_position = pose.position.x

            # Check if the block has reached or crossed x=0.0
            if block_x_position <= 0.0:
                stop_block(model_name, pose)
            else:
                # Only publish pose if the block is still moving
                publish_block_pose(model_name, pose)


def stop_block(model_name, current_pose):
    # Create a ModelState message to set the block's velocity to zero while maintaining its current pose
    block_state_msg = ModelState()
    block_state_msg.model_name = model_name
    block_state_msg.pose = current_pose  # Use the current pose
    block_state_msg.twist = Twist()  # Set velocity to zero
    block_state_msg.twist.linear.x = 0.0
    block_state_msg.twist.linear.y = 0.0
    block_state_msg.twist.linear.z = 0.0
    block_state_msg.twist.angular.x = 0.0
    block_state_msg.twist.angular.y = 0.0
    block_state_msg.twist.angular.z = 0.0
    try:
        resp = set_state_client(block_state_msg)
        if resp.success:
            rospy.loginfo(f"{model_name} stopped.")
            stopped_blocks[model_name] = 1
        else:
            rospy.logwarn(f"Failed to stop {model_name}.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to stop block failed: %s" % e)


def publish_block_pose(model_name, pose):
    block_pose_msg = ModelState()
    block_pose_msg.model_name = model_name
    block_pose_msg.pose = pose
    block_pose_publisher.publish(block_pose_msg)

if __name__ == '__main__':
    rospy.init_node('blocks_spawner')

    # Service clients
    spawn_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    wrench_client = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    set_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Publisher for current_blocks
    current_blocks_publisher = rospy.Publisher('current_blocks', Int8MultiArray, queue_size=1)
    current_blocks_msg = Int8MultiArray()
    current_blocks_msg.data = []
    block_pose_publisher = rospy.Publisher('block_poses', ModelState, queue_size=1)

    # Subscriber for model states
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

    # Wait for services to become available
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    rospy.wait_for_service('/gazebo/set_model_state')

    rospy.loginfo("All services are ready")

    # Get file path of blocks from parameter service
    red_box_path = rospy.get_param("/red_box_path")

    if not red_box_path:
        exit()
    else:
        rospy.loginfo(red_box_path + " has been extracted")

    with open(red_box_path, 'r') as f:
        red_xml_str = f.read()

    i = 0

    while not rospy.is_shutdown():
        index = int_to_string(i)
        y_position =  0.2  # random between -0.4 to 0.4
        rospy.loginfo("y position of new box: " + str(y_position))

        model_name = "red_blocks_" + index  # initialize model_name

        # Prepare model spawn request
        spawn_model_req = SpawnModel()
        spawn_model_req.model_name = model_name
        spawn_model_req.robot_namespace = model_name
        spawn_model_req.model_xml = red_xml_str
        spawn_model_req.initial_pose = Pose()
        spawn_model_req.initial_pose.position.x = 2
        spawn_model_req.initial_pose.position.y = y_position
        spawn_model_req.initial_pose.position.z = 0.2
        spawn_model_req.initial_pose.orientation.x = 0.0
        spawn_model_req.initial_pose.orientation.y = 0.0
        spawn_model_req.initial_pose.orientation.z = 0.0
        spawn_model_req.initial_pose.orientation.w = 1.0
        spawn_model_req.reference_frame = "world"

        try:
            spawn_model_resp = spawn_client(
            spawn_model_req.model_name,
            spawn_model_req.model_xml,
            spawn_model_req.robot_namespace,
            spawn_model_req.initial_pose,
            spawn_model_req.reference_frame
            )
            if spawn_model_resp.success:
            	rospy.loginfo(model_name + " has been spawned")
            else:
            	rospy.loginfo(model_name + " spawn failed")
            	
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            exit()

        # Prepare apply body wrench request
        apply_wrench_req = ApplyBodyWrench()
        apply_wrench_req.body_name = model_name + "::base_link"
        apply_wrench_req.reference_frame = "world"
        apply_wrench_req.wrench = Wrench()
        apply_wrench_req.wrench.force = Vector3()
        apply_wrench_req.wrench.force.x = -5.1
        apply_wrench_req.wrench.force.y = 0.0
        apply_wrench_req.wrench.force.z = 0.0
        apply_wrench_req.start_time = rospy.Time(0, 0)
        apply_wrench_req.duration = rospy.Duration(0, 1000000)

        # Call apply body wrench service
        try:
            apply_wrench_resp = wrench_client(
            body_name=apply_wrench_req.body_name,
            reference_frame=apply_wrench_req.reference_frame,
            wrench=apply_wrench_req.wrench,
            start_time=apply_wrench_req.start_time,
            duration=apply_wrench_req.duration
            )
            if apply_wrench_resp.success:
                rospy.loginfo(model_name + " speed initialized")
            else:
                rospy.loginfo(model_name + " fail to initialize speed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            exit()

        # Publish current blocks status
        current_blocks_publisher.publish(current_blocks_msg)

        # Loop end, increase index by 1
        i += 1
        rospy.loginfo("")

        rospy.sleep(20.0)  # frequency control, spawn one cylinder in each loop
        break;

