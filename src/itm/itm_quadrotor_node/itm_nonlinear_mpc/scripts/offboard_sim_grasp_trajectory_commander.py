#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-05-24 13:12:47
LastEditors: Wei Luo
LastEditTime: 2021-06-04 13:11:11
Note: Note
'''

import rospy
import numpy as np
from itm_nonlinear_mpc.msg import itm_trajectory_msg, itm_trajectory_point
from itm_nonlinear_mpc.srv import itm_trajectory_srv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time


# global parameters
robot_pose = np.array([])
got_robot_pose = False


def robot_odom_callback(msg):
    global robot_pose, got_robot_pose
    if not got_robot_pose:
        got_robot_pose = True
    robot_pose = np.array(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])


def robot_pose_callback(msg):
    global robot_pose, got_robot_pose
    if not got_robot_pose:
        got_robot_pose = True
    robot_pose = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


if __name__ == '__main__':
    rospy.init_node('uav_squared_trajectory')
    rospy.wait_for_service('/itm_trajectory_srv')
    set_traj_srv = rospy.ServiceProxy('itm_trajectory_srv', itm_trajectory_srv)
    robot_odom_sub_ = rospy.Subscriber(
        '/robot_pose', Odometry, robot_odom_callback)
    robot_pose_sub_ = rospy.Subscriber(
        '/vrpn_client_node/ITM_Q300/pose', PoseStamped, robot_pose_callback)

    trajectory_waypoints = np.array([
        [0.0, 0.0, 0.5],
        [0.0, 0.0, 0.8],
        [0.8, 0.0, 0.8],
        [0.8, 0.8, 0.8],
        [0.8, 0.0, 0.8],
        [0.0, 0.0, 0.8],
        [0.0, 0.0, 0.5],
    ])

    # main loop
    # trajectory_list = []
    msg_index = 0
    publish_index = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if (not got_robot_pose):
            rospy.logerr("no robot pose or odometry information received")
        else:
            if publish_index == msg_index:
                trajectory_list = []
                current_target = trajectory_waypoints[msg_index]
                point = itm_trajectory_point()
                point.x = current_target[0]
                point.y = current_target[1]
                point.z = current_target[2]
                point.yaw = 0.0
                point.derivative = 0
                point.fixed = True
                point.time_known = True
                point.time_stamp = 0.0
                trajectory_list.append(point)
                srv_obj = set_traj_srv(msg_index, trajectory_list)
                if srv_obj.success:
                    publish_index += 1
            # check if arrived the target pose
            pose_error = current_target.flatten() - robot_pose
            if (np.linalg.norm(pose_error) < 0.1):
                msg_index += 1
                if msg_index == 4:
                    time.sleep(10)
            else:
                print(np.linalg.norm(pose_error))
                print(current_target)
                print(robot_pose)
        if msg_index == trajectory_waypoints.shape[0]:
            break
        rate.sleep()
