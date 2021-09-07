#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-08-15 22:53:32
LastEditors: Wei Luo
LastEditTime: 2021-08-23 22:43:31
Note: this file provides a series of fixed point as
'''

import rospy
import rospkg
import sys
import numpy as np

from itm_mav_msgs.msg import itm_trajectory_msg, itm_trajectory_point
from itm_mav_srvs.srv import itm_trajectory_srv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# rospack = rospkg.RosPack()
# pack_path = rospack.get_path('itm_trajectory_planner')
# sys.path.append(pack_path + '/scripts')
# from poly_based_trajectory_planner import poly_t_trajectory as plt

# global parameters
mobilerobot_pose = np.array([])
uav_pose = np.array([])
got_mobilerobot_pose = False
got_mobilerobot_odom = False
got_uav_pose = False
got_uav_odom = False


def uav_odom_callback(msg):
    global uav_pose, got_uav_odom
    if not got_uav_odom:
        got_uav_odom = True
    uav_pose = np.array(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])


def uav_pose_callback(msg):
    global uav_pose, got_uav_pose
    if not got_uav_pose:
        got_uav_pose = True
    uav_pose = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


def mobilerobot_odom_callback(msg):
    global mobilerobot_pose, got_mobilerobot_odom
    if not got_mobilerobot_odom:
        got_mobilerobot_odom = True
    moblierobot_pose = np.array(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z + 0.5])


def mobilerobot_pose_callback(msg):
    global mobilerobot_pose, got_mobilerobot_pose
    if not got_mobilerobot_pose:
        got_mobilerobot_pose = True
    mobilerobot_pose = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


if __name__ == '__main__':
    rospy.init_node('uav_squared_trajectory')

    rospy.wait_for_service('/itm_trajectory_srv')
    set_traj_srv = rospy.ServiceProxy('itm_trajectory_srv', itm_trajectory_srv)
    uav_odom_sub_ = rospy.Subscriber(
        '/robot_pose', Odometry, uav_odom_callback)
    uav_pose_sub_ = rospy.Subscriber(
        '/vrpn_client_node/ITM_Q300/pose', PoseStamped, uav_pose_callback)
    landing_target_pose_sub_ = rospy.Subscriber(
        '/vrpn_client_node/l_hera/pose', PoseStamped, mobilerobot_pose_callback)
    landing_target_odom_sub_ = rospy.Subscriber(
        '/robotnik_base_control/odom', Odometry, mobilerobot_odom_callback)

    trajectory_waypoints = np.array([
        [0.0, 0.0, 0.5],
        [0.5, 0.0, 0.5],
        [1.5, 0.5, 0.6],
        [2.0, 1.5, 0.7],
        [3., 2.0, 0.8],
        [4., 2.5, 1.0],
        [4.5, 3., 1],
        [5, 3.3, 1],
        [5, 3.5, 0.9],
        [5, 3.8, 0.8],
        [5, 4, 0.6],
    ])

    timestamp_list = [
        1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 19.0, 21.
    ]

    # main loop
    trajectory_list = []
    msg_index = 0
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
        if (not got_uav_pose and not got_uav_odom):
            rospy.logerr("no robot pose or odometry information received")
        elif (got_uav_odom and got_uav_pose):
            rospy.logerr(
                "You've received both odom (sim) and pose (OptiTrack). That is impossible and unsupported! Stop and check your setup")
            break
        else:
            srv_obj = set_traj_srv(msg_index, trajectory_list)
            if srv_obj.success:
                break
    # # main loop
    # # trajectory_list = []
    # msg_index = 0
    # publish_index = 0
    # rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     if (not got_uav_pose and not got_uav_odom):
    #         rospy.logerr("no uav pose or odometry information received")
    #     elif (got_uav_odom and got_uav_pose):
    #         rospy.logerr(
    #             "You've received both odom (sim) and pose (OptiTrack). That is impossible and unsupported! Stop and check your setup")
    #         break
    #     else:
    #         if publish_index == msg_index:
    #             trajectory_list = []
    #             current_target = trajectory_waypoints[msg_index]
    #             point = itm_trajectory_point()
    #             point.x = current_target[0]
    #             point.y = current_target[1]
    #             point.z = current_target[2]
    #             point.vx = 0.0
    #             point.vy = 0.0
    #             point.vz = 0.0
    #             point.roll = 0.0
    #             point.pitch = 0.0
    #             point.yaw = 0.0
    #             point.derivative = 0
    #             point.fixed = True
    #             point.time_known = True
    #             point.time_stamp = 0.0
    #             trajectory_list.append(point)
    #             srv_obj = set_traj_srv(msg_index, trajectory_list)
    #             if srv_obj.success:
    #                 publish_index += 1
    #         # check if arrived the target pose
    #         pose_error = current_target.flatten() - uav_pose
    #         if (np.linalg.norm(pose_error) < 0.1):
    #             msg_index += 1
    #         else:
    #             print(np.linalg.norm(pose_error))
    #             print(current_target)
    #             print(uav_pose)
    #     if msg_index == trajectory_waypoints.shape[0]:
    #         break
    #     rate.sleep()
