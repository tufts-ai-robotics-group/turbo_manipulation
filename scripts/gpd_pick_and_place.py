#!/usr/bin/env python

# Gyan Tatiya

import os
import time
import math
import shutil
import numpy as np
from datetime import datetime

import rospy
from gpd_ros.msg import GraspConfigList
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped, PoseStamped, Pose, Quaternion
from joint_recorder.srv import recorderSrv, recorderSrvRequest
from robotiq_85_msgs.msg import GripperCmd
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, JointState

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

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
        self.gripper_topic = '/gripper/joint_states'
        self.wrench_topic = '/right/wrench'

        self.sensor_data_path = '/home/pc1/Downloads/datasets/temp/'

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
        self.object_detector = rospy.ServiceProxy('/turbo_object_detector/detect', TabletopPerception)
        self.recording_service = rospy.ServiceProxy('/data_recording_service', recorderSrv)
        self.recording_srv_request = recorderSrvRequest()  # Create a service request object

        self.arm_group = moveit_commander.MoveGroupCommander("right_manipulator")  # Creating moveit client to control arm
        self.gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=1)

        # Performs transformation from one link to other
        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer)  # Listens to tf to get transformation information necessary
        rospy.sleep(2)  # Provides time for buffer to collect data

        # self.initialise_robot()
        self.open_gripper()

        # Wait for grasps to arrive.
        # rate = rospy.Rate(1)

        grasp_count = 0
        while not rospy.is_shutdown():

            self.grasp_pose_done = False
            self.grasp_info_done = False

            self.detect_object()
            
            x = 0
            while self.grasp_pose_done == False:
                time.sleep(0.1)
                x = x + 1
                if x > 300:  # Adds a 120 second time out for the while loop
                    print("Error: Was not able to properly receive and process information")
                    break

            if self.grasp_pose_done:
                pose = self.filter_grasp_pose()
                if pose:
                    print('MOVING to pose: ', pose)

                    sensorDataPath = self.sensor_data_path + 'trial-' + str(grasp_count) + '_' + datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + '/'

                    # raw_input('Approach ...')
                    pose.pose.position.z += 0.10
                    self.open_gripper()
                    self.move_safely(pose)

                    # raw_input('Grasp ...')
                    self.start_recording(sensorDataPath + 'grasp' + '/')
                    pose.pose.position.z -= 0.10
                    self.move_safely(pose)
                    self.close_gripper()
                    self.stop_recording()
                    gripper_pose = self.get_gripper_position()
                    print('gripper_pose: ', gripper_pose)

                    # raw_input('Lift ...')
                    self.start_recording(sensorDataPath + 'lift' + '/')
                    pose.pose.position.z += 0.10
                    self.move_safely(pose)
                    self.stop_recording()
                    
                    # Make sure grasp was successful
                    if not (self.check_grasp_gripper_pose() and self.check_grasp_gripper_pose_difference(gripper_pose)):
                        print('Retrying from top ...')
                        shutil.rmtree(sensorDataPath)
                        continue

                    # raw_input('Move to a random location ...')
                    success = False
                    while not success:
                        pose.pose.position.x = np.random.uniform(self.table_corners_x[0] + 0.15, self.table_corners_x[1] - 0.15)
                        pose.pose.position.y = np.random.uniform(self.table_corners_y[0] + 0.1, self.table_corners_y[1] - 0.1)
                        success = self.check_plan(pose, num_points=10)
                    self.move_safely(pose)

                    # raw_input('Place ...')
                    self.start_recording(sensorDataPath + 'place' + '/')
                    pose.pose.position.z -= 0.10
                    self.move_safely(pose)
                    self.open_gripper()
                    self.stop_recording()
                    pose.pose.position.z += 0.20
                    self.move_safely(pose)

                    grasp_count += 1

                # time.sleep(10)
    
    def detect_object(self):
        print("Waiting for service ... ")
        # time.sleep(5.0)
        # print("Delay Ends ... ")

        # os.system('rosservice call /turbo_object_detector/detect')        
        # try:
        #     rospy.wait_for_service('/turbo_object_detector/detect', timeout=rospy.Duration(10.0))
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s" % e)
        
        # try:
        #     proxy = rospy.ServiceProxy('/turbo_object_detector/detect', TabletopPerception)
        #     resp1 = proxy()
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s" % e)

        resp = self.object_detector()
        print("Objects Found : ", len(resp.cloud_clusters))

        if len(resp.cloud_clusters) > 0:
            self.filtered_pointcloud_pub.publish(np.random.choice(resp.cloud_clusters))

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
    
    def filter_grasp_pose(self):
        '''
        TODO:
        - select pose that is close to the current arm pose        
        '''

        down_orientation = quaternion_from_euler(0, 3.1415 / 2, 0)

        filtered_grasp_poses = []
        for pose in self.grasp_pose_array:

            angle = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y,
                                           pose.pose.orientation.z, pose.pose.orientation.w])
            roll, pitch, yaw = math.degrees(angle[0]), math.degrees(angle[1]), math.degrees(angle[2])
            # print('pitch: ', angle[1], pitch)

            # if (self.table_corners_x[0] + 0.05 < pose.pose.position.x < self.table_corners_x[1] - 0.05) \
            #     and (self.table_corners_y[0] + 0.05 < pose.pose.position.y < self.table_corners_y[1] - 0.05) \
            #     and (self.table_height_z < pose.pose.position.z < self.table_height_z + 0.1) \
            #     and ((45 < pitch < 135) or (-315 < pitch < -225)):
            if ((45 < pitch < 135) or (-315 < pitch < -225)):
                
                # print('pitch: ', angle[1], pitch)
                filtered_grasp_poses.append([pose, abs(90 - pitch)])
        
        filtered_grasp_poses = sorted(filtered_grasp_poses, key=lambda pose: pose[1])
        
        if len(filtered_grasp_poses) > 0:
            for pose, pitch_diff in filtered_grasp_poses:

                self.pub_coordinates.publish(pose)
                print("Best pose (within limits): ", pose)

                pose.pose.orientation.x = down_orientation[0]
                pose.pose.orientation.y = down_orientation[1]
                pose.pose.orientation.z = down_orientation[2]
                pose.pose.orientation.w = down_orientation[3]

                pose.pose.position.z += 0.20  # adding gripper length

                pose.pose.position.z += 0.10
                # Check if approach pose is plannable
                if self.check_plan(pose, num_points=10):
                    # Check if pose is plannable
                    pose.pose.position.z -= 0.10
                    if self.check_plan(pose, num_points=10):
                        return pose
        else:
            return None
    
    def perform_transformation(self, pose):

        target_transform = self.buffer.lookup_transform(self.world_tf, self.camera_tf, rospy.Time(0))
        pose = tf2_geometry_msgs.do_transform_pose(pose, target_transform)

        return pose
    
    def check_plan(self, pose, num_points=3):

        self.arm_group.set_pose_target(pose, end_effector_link="right_gripper_base_link")  # Set target pose
        robot_trajectory = self.arm_group.plan()  # Plan motion and return whether point is viable or not/calculation time

        # print("robot_trajectory: ", len(robot_trajectory.joint_trajectory.points))

        success = False
        if 0 < len(robot_trajectory.joint_trajectory.points) <= num_points:
            success = True
            # print("robot_trajectory: ", robot_trajectory.joint_trajectory)
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
        print('moving safely')
        while not success:
            success = self.check_plan(pose)
            if not success:
                print('moving to mid point')
                curr_pose = self.arm_group.get_current_pose().pose
                # print('curr_pose: ', curr_pose)

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
    
    def get_gripper_position(self):

        js = rospy.wait_for_message(self.gripper_topic, JointState)

        return round(js.position[0], 2)
    
    def get_force_values(self):

        w_msg = rospy.wait_for_message(self.wrench_topic, WrenchStamped)

        return w_msg.wrench.force
    
    def check_grasp_gripper_pose(self):
        # Check if gripper pose is between open and closed completely
        
        gripper_pos = self.get_gripper_position()
        print('Gripper pos: ', gripper_pos)
        
        grasp_success = 0 < gripper_pos < 0.79
        if not grasp_success:
            print('Grasp unsuccessful: gripper_pose')

        return grasp_success
    
    def check_grasp_gripper_pose_difference(self, prev_gripper_pose):
        # Close gripper again and check if there is a difference in the prior 
        # and new gripper pose after closing

        self.close_gripper()
        gripper_pos = self.get_gripper_position()
        gripper_pose_diff = abs(prev_gripper_pose - gripper_pos)
        print('gripper_pose_diff: ', gripper_pose_diff)

        grasp_success = gripper_pose_diff < 0.1
        if not grasp_success:
            print('Grasp unsuccessful: pose_difference')

        return grasp_success
    
    def start_recording(self, folder_name):
        command1 = "mkdir -p " + folder_name + "camera_rgb_image/"
        command2 = "mkdir -p " + folder_name + "camera_depth_image/"
        # command3 = "mkdir -p " + folder_name + "touch_image/"
        command4 = "mkdir -p " + folder_name + "joint_states/"
        command5 = "mkdir -p " + folder_name + "gripper_joint_states/"
        command6 = "mkdir -p " + folder_name + "wrench/"
        command7 = "mkdir -p " + folder_name + "audio/"
        os.system(command1)
        os.system(command2)
        # os.system(command3)
        os.system(command4)
        os.system(command5)
        os.system(command6)
        os.system(command7)

        self.recording_srv_request.command.data = "set_file_name"
        self.recording_srv_request.fileName.data = folder_name + "joint_states/joint_states.csv"
        self.recording_srv_request.topic.data = "/right/joint_states"
        self.recording_service.call(self.recording_srv_request)

        self.recording_srv_request.command.data = "set_file_name"
        self.recording_srv_request.fileName.data = folder_name + "gripper_joint_states/gripper_joint_states.csv"
        self.recording_srv_request.topic.data = "/gripper/joint_states"
        self.recording_service.call(self.recording_srv_request)

        self.recording_srv_request.command.data = "set_file_name"
        self.recording_srv_request.fileName.data = folder_name + "wrench/wrench.csv"
        self.recording_srv_request.topic.data = "/right/wrench"
        self.recording_service.call(self.recording_srv_request)

        self.recording_srv_request.command.data = "set_file_name"
        self.recording_srv_request.fileName.data = folder_name + "audio/audio.wav"
        self.recording_srv_request.topic.data = "audio_capture"
        self.recording_service.call(self.recording_srv_request)

        self.recording_srv_request.command.data = "set_file_name"
        self.recording_srv_request.fileName.data = folder_name + "camera_rgb_image/"
        self.recording_srv_request.topic.data = "color_frame_capture"
        self.recording_service.call(self.recording_srv_request)

        self.recording_srv_request.command.data = "set_file_name"
        self.recording_srv_request.fileName.data = folder_name + "camera_depth_image/"
        self.recording_srv_request.topic.data = "depth_frame_capture"
        self.recording_service.call(self.recording_srv_request)

        # self.recording_srv_request.command.data = "set_file_name"
        # self.recording_srv_request.fileName.data = folder_name + "touch_image/"
        # self.recording_srv_request.topic.data = "touch_frame_capture"
        # self.recording_service.call(self.recording_srv_request)

        self.recording_srv_request.command.data = "start"
        self.recording_service.call(self.recording_srv_request)

    
    def stop_recording(self):

        self.recording_srv_request.command.data = "stop"
        self.recording_service.call(self.recording_srv_request)


if __name__ == "__main__":
    GPDPickAndPlace()

    '''
    TODO:
    - Restrict planning trajectory to avoid weird plans
        X Restrict joint angles
        - Observe weird plans and find what makes them weird
            X Try to reduce that
            - Make sure all the points in the plan are inside table top; use fk

    X Filter point cloud before passing to gpd
    X Check if grasp was successful or not
    '''
