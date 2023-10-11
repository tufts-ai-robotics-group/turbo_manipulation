#!/usr/bin/env python

# Gyan Tatiya
import os
import time
import math
import rospy
from gpd_ros.msg import GraspConfigList
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from robotiq_85_msgs.msg import GripperCmd

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from turbo_robot_vision.srv import *


def quaternion_to_matrix(quaternion):
    # Function to convert quaternion to rotation matrix

    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    rotation_matrix = np.zeros((3, 3))
    
    rotation_matrix[0, 0] = 1 - 2*y*y - 2*z*z
    rotation_matrix[0, 1] = 2*x*y - 2*z*w
    rotation_matrix[0, 2] = 2*x*z + 2*y*w
    
    rotation_matrix[1, 0] = 2*x*y + 2*z*w
    rotation_matrix[1, 1] = 1 - 2*x*x - 2*z*z
    rotation_matrix[1, 2] = 2*y*z - 2*x*w
    
    rotation_matrix[2, 0] = 2*x*z - 2*y*w
    rotation_matrix[2, 1] = 2*y*z + 2*x*w
    rotation_matrix[2, 2] = 1 - 2*x*x - 2*y*y
    
    return rotation_matrix

def move_pose_forward(pose, distance):
    # Function to move pose forward by a given distance in its orientation

    rotation_matrix = quaternion_to_matrix(pose.pose.orientation)
    translation_vector = np.array([distance, 0, 0])  # Update x by 'distance'

    # Apply the rotation matrix to the translation vector
    updated_translation = np.dot(rotation_matrix, translation_vector)

    # Update the pose
    pose.pose.position.x += updated_translation[0]
    pose.pose.position.y += updated_translation[1]
    pose.pose.position.z += updated_translation[2]

    return pose


class GPDPickAndPlace:
    def __init__(self):

        # Create a ROS node.
        rospy.init_node('get_grasps')

        self.world_tf = "world"
        self.camera_tf = "camera_depth_optical_frame"

        # Table corner poses:
        self.table_corners_x = [0.038783, 0.87016]
        self.table_corners_y = [0.031553, 0.46748]
        self.table_height_z = 0.723905 + 0.006095

        self.grasp_pose_array = []
        self.grasp_info_array = []

        rospy.Subscriber(
            "/detect_grasps/plot_grasps",
            MarkerArray,
            self.callback_coordinates,
            queue_size=100,
        )  # Crate publishers and subscribers
        # rospy.Subscriber(
        #     "detect_grasps/clustered_grasps",
        #     GraspConfigList,
        #     self.callback_grasp_info,
        #     queue_size=100,
        # )
        self.pub_coordinates = rospy.Publisher("grasp_pose", PoseStamped, queue_size=1000)
        self.filtered_pointcloud_pub = rospy.Publisher("filtered_pointcloud", PointCloud2, queue_size=10)

        self.arm_group = moveit_commander.MoveGroupCommander("right_manipulator")  # Creating moveit client to control arm
        self.gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=1)

        # Performs transformation from one link to other
        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer)  # Listens to tf to get transformaton information necessary
        rospy.sleep(2)  # Provides time for buffer to collect data

        # self.initialise_robot()
        self.open_gripper()

        self.proxy = rospy.ServiceProxy('/turbo_object_detector/detect', TabletopPerception)
        # Wait for grasps to arrive.
        # rate = rospy.Rate(1) 

        while not rospy.is_shutdown():

            self.grasp_pose_done = False
            self.grasp_info_done = False

            self.detect_object()
            
            x = 0
            while self.grasp_pose_done == False:
                time.sleep(0.1)
                x = x + 1
                if x > 1200:  # Adds a 120 second time out for the while loop
                    print(
                        "Error: Was not able to properly recieve and process information"
                    )
                    break

            
            if self.grasp_pose_done:
                # x = 0
                success_array = (
                    []
                )  # Stores whether grasp suceeds in inverse kinematics test or not
                for grasp_pose in self.grasp_pose_array:
                    success = self.IK_verification(grasp_pose)
                    success_array.append(success)
                    if success:
                        # rospy.signal_shutdown("Exiting ROS node")
                        break
                    # x = x + 1
                # rospy.loginfo("Success: " + str(success_array))
                time.sleep(10)
    
    def detect_object(self):
        print("Waiting for service ... ")
        time.sleep(5.0)
        print("Delay Ends ... ")

        # os.system('rosservice call /turbo_object_detector/detect')        
        try:
            rospy.wait_for_service('/turbo_object_detector/detect', timeout=rospy.Duration(5.0))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        resp1 = self.proxy()
        print("Objects Found : ", len(resp1.cloud_clusters))
        
        self.filtered_pointcloud_pub.publish(resp1.cloud_clusters[0])

    def callback_coordinates(self, msg):
        # Stores grasp coordinate information from GPD
        self.grasp_pose_array = []

        i = 0
        while i < (len(msg.markers) / 4):
            # Message has 4 markers for each part of end effector. We only need to look at one part per pose
            marker = msg.markers[3 + (i * 4)]  # Iterate through grasp info and only look at last part which is the base of end effector
            # Extract the quaternion components from the marker message
            quaternion = (marker.pose.orientation.x, marker.pose.orientation.y,
                          marker.pose.orientation.z, marker.pose.orientation.w)

            pose = PoseStamped()
            pose.header.frame_id = self.world_tf
            pose.pose.orientation = Quaternion(*quaternion)
            pose.pose.position = marker.pose.position
            
            i = i + 1

            # pose = self.perform_transformation(pose)
            pose = move_pose_forward(pose, 0.03)
            self.grasp_pose_array.append(pose)

            # self.pub_coordinates.publish(pose)

        self.grasp_pose_done = True

        print('self.grasp_pose_array: ', len(self.grasp_pose_array))
    
    def callback_grasp_info(self, msg):
        # Stores grasp score in an array
        self.grasp_info_array = []
        
        for grasp in msg.grasps:
            score = grasp.score
            self.grasp_info_array.append(score)

        self.grasp_info_done = True

        print('self.grasp_info_array: ', len(self.grasp_info_array))
    
    def IK_verification(self, pose):
        '''
        TODO:
        - select pose that is close to the currecnt arm pose        
        '''

        down_orientation = quaternion_from_euler(0, 3.1415 / 2, 0)
        # pose.pose.orientation.x = down_orientation[0]
        # pose.pose.orientation.y = down_orientation[1]
        # pose.pose.orientation.z = down_orientation[2]
        # pose.pose.orientation.w = down_orientation[3]

        angle = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y,
                                       pose.pose.orientation.z, pose.pose.orientation.w])
        roll, pitch, yaw = math.degrees(angle[0]), math.degrees(angle[1]), math.degrees(angle[2])
        print('pitch: ', angle[1], pitch)

        success = False
        # if (self.table_corners_x[0] + 0.05 < pose.pose.position.x < self.table_corners_x[1] - 0.05) \
        #     and (self.table_corners_y[0] + 0.05 < pose.pose.position.y < self.table_corners_y[1] - 0.05) \
        #     and (self.table_height_z < pose.pose.position.z < self.table_height_z + 0.1) \
        #     and ((45 < pitch < 135) or (-315 < pitch < -225)):
        if ((45 < pitch < 135) or (-315 < pitch < -225)):
            
            # print('pitch: ', angle[1], pitch)

            self.pub_coordinates.publish(pose)
            # print("pose (within limits): ", pose)

            pose.pose.orientation.x = down_orientation[0]
            pose.pose.orientation.y = down_orientation[1]
            pose.pose.orientation.z = down_orientation[2]
            pose.pose.orientation.w = down_orientation[3]
            
            pose.pose.position.y += 0.01  # offset
            pose.pose.position.z += 0.20  # adding gripper length
            
            # if self.check_plan(pose):
            #     pose.pose.position.z += 0.10
            #     success = self.check_plan(pose)
            pose.pose.position.z += 0.10

            # if success:
            print('FINAL pose: ', pose)
            self.open_gripper()
            # raw_input('Approach ...')
            # self.arm_group.go(wait=True)
            self.move_safely(pose)

            # raw_input('Grasp ...')
            pose.pose.position.z -= 0.10
            self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
            # self.arm_group.go(wait=True)
            self.move_safely(pose)
            self.close_gripper()

            # raw_input('Lift ...')
            pose.pose.position.z += 0.10
            self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
            # self.arm_group.go(wait=True)
            self.move_safely(pose)

            success = False
            while not success:
                pose.pose.position.x = np.random.uniform(self.table_corners_x[0] + 0.15, self.table_corners_x[1] - 0.15)
                pose.pose.position.y = np.random.uniform(self.table_corners_y[0] + 0.1, self.table_corners_y[1] - 0.1)
                success = self.check_plan(pose)

            # raw_input('Move to a random location ...')
            self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
            # self.arm_group.go(wait=True)
            self.move_safely(pose)

            # raw_input('Place ...')
            pose.pose.position.z -= 0.10
            self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
            # self.arm_group.go(wait=True)
            self.move_safely(pose)
            self.open_gripper()
            pose.pose.position.z += 0.20
            self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
            # self.arm_group.go(wait=True)
            self.move_safely(pose)

            # self.initialise_robot()

        return success
    
    def perform_transformation(self, pose):

        target_transform = self.buffer.lookup_transform(self.world_tf, self.camera_tf, rospy.Time(0))
        pose = tf2_geometry_msgs.do_transform_pose(pose, target_transform)

        return pose
    
    def check_plan(self, pose):

        self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
        robot_trajectory = self.arm_group.plan()  # Plan motion and return whether point is viable or not/calculation time

        print("robot_trajectory: ", len(robot_trajectory.joint_trajectory.points))

        success = False
        if 0 < len(robot_trajectory.joint_trajectory.points) <= 3:
            success = True
        else:
            # It is always good to clear your targets after planning with poses
            self.arm_group.clear_pose_targets()
        
        print('success: ', success)

        return success
    
    def move_safely(self, pose):

        downOrientation = quaternion_from_euler(0, 3.1415 / 2, 0)
        pose.pose.orientation.x = downOrientation[0]
        pose.pose.orientation.y = downOrientation[1]
        pose.pose.orientation.z = downOrientation[2]
        pose.pose.orientation.w = downOrientation[3]
        
        success = False
        print('GT 1')
        while not success:
            success = self.check_plan(pose)
            print('GT 2')
            if not success:
                print('GT 3')
                curr_pose = self.arm_group.get_current_pose().pose
                print('curr_pose: ', curr_pose)

                curr_x, curr_y, curr_z = curr_pose.position.x, curr_pose.position.y, curr_pose.position.z

                curr_pose.position.x = (curr_x + pose.pose.position.x) / 2
                curr_pose.position.y = (curr_y + pose.pose.position.y) / 2
                curr_pose.position.z = (curr_z + pose.pose.position.z) / 2

                curr_pose2 = PoseStamped()
                curr_pose2.header.frame_id = self.world_tf
                curr_pose2.pose.orientation = curr_pose.orientation
                curr_pose2.pose.position = curr_pose.position

                print('curr_pose2: ', curr_pose2)

                self.move_safely(curr_pose2)
            else:
                # raw_input('Go ...')
                self.arm_group.go(wait=True)
    
    def initialise_robot(self):

        self.open_gripper()
        pose = self.arm_group.get_current_pose().pose

        # Block point
        pose.position.x = -0.1
        pose.position.y = 0.3
        pose.position.z = 1.2

        down_orientation = quaternion_from_euler(0, 3.1415 / 2, 0)
        pose.orientation.x = down_orientation[0]
        pose.orientation.y = down_orientation[1]
        pose.orientation.z = down_orientation[2]
        pose.orientation.w = down_orientation[3]

        # success = self.check_plan(pose)
        raw_input('Initialising robot pose ...')

        # if success:
        #     self.arm_group.go(wait=True)
        
        self.move_safely(pose)
    
    def open_gripper(self):

        msg = GripperCmd(position=0.1, speed=1, force=100.0)
        rospy.sleep(1)
        self.gripper_pub.publish(msg)
        rospy.sleep(1)

    def close_gripper(self):

        msg = GripperCmd(position=0.00, speed=1, force=100.0)
        rospy.sleep(1)
        self.gripper_pub.publish(msg)
        rospy.sleep(1)


if __name__ == "__main__":
    GPDPickAndPlace()

    '''
    TODO:
    - Restrict planning trajectory to avoid weird plans
        X Restrict joint angles
        - Observe weird plans and find what makes them weird
            X Try to reduce that
            - Make sure all the points in the plan are inside table top; use fk

    - Filter point cloud before passing to gpd
    - Check if grasp was successful or not
    '''
