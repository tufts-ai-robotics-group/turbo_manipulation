#!/usr/bin/env python

# Gyan Tatiya

import rospy
import moveit_commander
import tf
import rospkg
import os

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

    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)

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
