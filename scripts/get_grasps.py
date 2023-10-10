#!/usr/bin/env python

# Gyan Tatiya

import rospy
# from grasp_candidates_classifier.msg import GraspConfigList
from gpd_ros.msg import GraspConfigList

def callback(msg):
    global grasps
    grasps = msg.grasps


if __name__ == "__main__":
    grasps = []

    # Create a ROS node.
    rospy.init_node('get_grasps')

    # Subscribe to the ROS topic that contains the grasps.
    sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

    # Wait for grasps to arrive.
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():    
        if len(grasps) > 0:
            rospy.loginfo('Received %d grasps.', len(grasps))
            break
        rate.sleep()
