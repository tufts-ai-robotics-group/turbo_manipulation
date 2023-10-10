#!/usr/bin/env python

# Gyan Tatiya

import rospy
import moveit_commander
import tf
import rospkg
import os
import time

from robotiq_85_msgs.msg import GripperCmd

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


def check_plan(pose):

    arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
    robot_trajectory = arm_group.plan()  # Plan motion and return whether point is viable or not/calculation time

    print("robot_trajectory: ", len(robot_trajectory.joint_trajectory.points))

    if len(robot_trajectory.joint_trajectory.points) == 0:
        # It is always good to clear your targets after planning with poses
        arm_group.clear_pose_targets()

    success = False
    if 0 < len(robot_trajectory.joint_trajectory.points) <= 6:
        success = True
    
    print('success: ', success)

    return success


def move_safely(pose):

    downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415 / 2, 0)
    pose.orientation.x = downOrientation[0]
    pose.orientation.y = downOrientation[1]
    pose.orientation.z = downOrientation[2]
    pose.orientation.w = downOrientation[3]
    
    success = False
    while not success:
        success = check_plan(pose)
        if not success:
            curr_pose = arm_group.get_current_pose().pose
            # print('curr_pose: ', curr_pose)
            curr_x, curr_y, curr_z = curr_pose.position.x, curr_pose.position.y, curr_pose.position.z

            curr_pose.position.x = (curr_x + pose.position.x) / 2
            curr_pose.position.y = (curr_y + pose.position.y) / 2
            curr_pose.position.z = (curr_z + pose.position.z) / 2
            # print('new pose: ', curr_pose)

            move_safely(curr_pose)
        else:
            raw_input('Go ...')
            arm_group.go(wait=True)


if __name__ == "__main__":
    """
    This script shows how to control real UR5's arm and gripper
    Launch this first:
    roslaunch turbo_bringup right_arm.launch
    """

    rospy.init_node('real_moveit_example', anonymous=True)

    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("right_manipulator")  # Creating moveit client to control arm
    gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=1)

    pub_coordinates = rospy.Publisher("grasp_pose", PoseStamped, queue_size=1000)

    # arm_group.set_planner_id("LBKPIECE")

    # You can get the reference frame for a certain group by executing this line:
    print("Arm Reference frame: %s" % arm_group.get_planning_frame())

    # You can get the end-effector link for a certaing group executing this line:
    print("Arm End effector: %s" % arm_group.get_end_effector_link())

    # You can get a list with all the groups of the robot like this:
    print("Robot Groups:", robot.get_group_names())

    # You can get the current values of the joints like this:
    print("Arm Current Joint Values:", arm_group.get_current_joint_values())

    # You can also get the current Pose of the end-effector of the robot like this:
    print("Arm Current Pose:")
    print(arm_group.get_current_pose())

    # Finally, you can check the general status of the robot like this:
    print("Robot State:")
    print(robot.get_current_state())

    # Open gripper
    msg = GripperCmd(position=0.1, speed=1, force=100.0)
    rospy.sleep(1)
    gripper_pub.publish(msg)
    rospy.sleep(1)

    # How to go to a pose using moveit:
    pose = arm_group.get_current_pose().pose

    # Block point
    pose.position.x = 0.2
    pose.position.y = 0.3
    pose.position.z = 0.95

    downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415 / 2, 0)
    print("downOrientation: ", downOrientation)
    pose.orientation.x = downOrientation[0]
    pose.orientation.y = downOrientation[1]
    pose.orientation.z = downOrientation[2]
    pose.orientation.w = downOrientation[3]

    pose2 = PoseStamped()
    pose2.header.frame_id = 'world'
    pose2.pose.position = pose.position
    pose2.pose.orientation = Quaternion(*downOrientation)
    pub_coordinates.publish(pose2)

    move_safely(pose)
    '''
    arm_group.set_pose_target(pose)
    plan_output = arm_group.plan()  # Plan motion and return whether point is viable or not/calculation time
    arm_group.go(wait=True)
    '''

    # Close gripper
    msg = GripperCmd(position=0.00, speed=1, force=100.0)
    rospy.sleep(1)
    gripper_pub.publish(msg)
    rospy.sleep(1)

    # How to go to a joint positions using moveit:
    # [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    point = [-0.78, -1.33, 2.26, -1.84, 2.13, 2.48]
    arm_group.set_joint_value_target(point)
    arm_group.go(wait=True)

    # Open gripper
    msg = GripperCmd(position=0.1, speed=1, force=100.0)
    rospy.sleep(1)
    gripper_pub.publish(msg)
    rospy.sleep(1)
