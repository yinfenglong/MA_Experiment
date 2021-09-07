#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-07-31 20:42:56
LastEditors: Wei Luo
LastEditTime: 2021-08-15 23:20:25
Note: Note
'''

import rospy
import numpy as np
from itm_mav_msgs.msg import itm_trajectory_msg, itm_trajectory_point
from itm_mav_srvs.srv import itm_trajectory_srv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


# global parameters
robot_pose = np.array([])
got_robot_pose = False
got_robot_odom = False


def robot_odom_callback(msg):
    global robot_pose, got_robot_odom
    if not got_robot_odom:
        got_robot_odom = True
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
        [0.5, 0.0, 0.5],
        [0.5, 0.5, 0.5],
        [0.0, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
        [-0.5, 0.0, 0.5],
        [-0.5, -0.5, 0.5],
        [0.0, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.0, 0.5],
        [0.0, 0.0, 0.5],
    ])

    timestamp_list = [
        1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 19.0, 20.0, 24.0,
    ]

    # main loop
    trajectory_list = []
    msg_index = 0
    publish_index = 0

    for idx in range(trajectory_waypoints.shape[0]):

        point = itm_trajectory_point()
        point.x = trajectory_waypoints[idx, 0]
        point.y = trajectory_waypoints[idx, 1]
        point.z = trajectory_waypoints[idx, 2]
        point.yaw = 0.0
        point.derivative = 0
        point.fixed = True
        point.time_known = True
        point.time_stamp = timestamp_list[idx]
        trajectory_list.append(point)

    while not rospy.is_shutdown():
        if (not got_robot_pose and not got_robot_odom):
            rospy.logerr("no robot pose or odometry information received")
        elif (got_robot_odom and got_robot_pose):
            rospy.logerr(
                "You've received both odom (sim) and pose (OptiTrack). That is impossible and unsupported! Stop and check your setup")
            break
        else:
            srv_obj = set_traj_srv(msg_index, trajectory_list)
            if srv_obj.success:
                break
