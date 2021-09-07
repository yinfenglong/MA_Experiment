#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-07-31 20:42:56
LastEditors: Wei Luo
LastEditTime: 2021-08-17 15:51:54
Note: Note
'''

import os
import numpy as np

import rospy
from itm_mav_msgs.msg import itm_trajectory_msg, itm_trajectory_point
from itm_mav_srvs.srv import itm_trajectory_srv
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


class TrajectoryPublisher(object):
    def __init__(self, horizon=20, publish_rate=100, mpc_dt=0.1, desired_trajectory=None):
        self.robot_pose = np.array([])
        self.got_robot_pose = False
        self.got_robot_odom = False
        self.robot_odom_sub_ = rospy.Subscriber(
            '/robot_pose', Odometry, self.robot_odom_callback)
        self.robot_pose_sub_ = rospy.Subscriber(
            '/vrpn_client_node/ITM_Q250/pose', PoseStamped, self.robot_pose_callback)
        self.trajectory_pub = rospy.Publisher(
            '/robot_trajectory', itm_trajectory_msg, queue_size=50)
        self.publish_rate = publish_rate
        self.rate = rospy.Rate(publish_rate)
        self.time_out_counter = 0
        self.arrive_first_waypoint = False
        self.horizon = horizon
        self.traj_iter = 0
        self.traj_max_iter = desired_trajectory.shape[0]
        self.mpc_dt = mpc_dt  # mpc defined dt in the MPC code
        self.desired_trajectory = desired_trajectory

    def robot_odom_callback(self, msg):
        if not self.got_robot_odom:
            self.got_robot_odom = True
        self.robot_pose = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def robot_pose_callback(self, msg):
        if not self.got_robot_pose:
            self.got_robot_pose = True
        self.robot_pose = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def go_to_first_point(self, ):
        pose_error = self.desired_trajectory[0, :3].flatten() - self.robot_pose
        if (np.linalg.norm(pose_error) < 0.1):
            self.arrive_first_waypoint = True
            rospy.sleep(3)
        else:
            trajectories_list = itm_trajectory_msg()
            trajectories_list.header = Header()
            trajectories_list.traj = []
            trajectories_list.size = self.horizon + 1
            for i in range(self.horizon + 1):
                temp_ = itm_trajectory_point()
                temp_.x = self.desired_trajectory[0, 0]
                temp_.y = self.desired_trajectory[0, 1]
                temp_.z = self.desired_trajectory[0, 2]
                temp_.vx = self.desired_trajectory[0, 6]
                temp_.vy = self.desired_trajectory[0, 7]
                temp_.vz = self.desired_trajectory[0, 8]
                temp_.yaw = self.desired_trajectory[0, 5]
                temp_.roll = self.desired_trajectory[0, 3]
                temp_.pitch = self.desired_trajectory[0, 4]
                quaternion_ = self.rpy_to_quaternion(
                    np.array([temp_.roll, temp_.pitch, temp_.yaw]))
                temp_.q[0] = quaternion_[0]
                temp_.q[1] = quaternion_[1]
                temp_.q[2] = quaternion_[2]
                temp_.q[3] = quaternion_[3]
                temp_.quaternion_given = True
                temp_.roll_rate_des = 0.0
                temp_.pitch_rate_des = 0.0
                temp_.yaw_rate_des = 0.0
                temp_.thrust_des = 9.8066
                temp_.input_given = True
                trajectories_list.traj.append(temp_)
            self.trajectory_pub.publish(trajectories_list)

    def stop(self):
        trajectories_list = itm_trajectory_msg()
        trajectories_list.header = Header()
        trajectories_list.traj = []
        trajectories_list.size = self.horizon + 1
        for i in range(self.horizon + 1):
            temp_ = itm_trajectory_point()
            temp_.x = self.desired_trajectory[-1, 0]
            temp_.y = self.desired_trajectory[-1, 1]
            temp_.z = self.desired_trajectory[-1, 2]
            temp_.vx = self.desired_trajectory[-1, 6]
            temp_.vy = self.desired_trajectory[-1, 7]
            temp_.vz = self.desired_trajectory[-1, 8]
            temp_.roll = self.desired_trajectory[-1, 3]
            temp_.pitch = self.desired_trajectory[-1, 4]
            temp_.yaw = self.desired_trajectory[-1, 5]
            quaternion_ = self.rpy_to_quaternion(
                np.array([temp_.roll, temp_.pitch, temp_.yaw]))
            temp_.q[0] = quaternion_[0]
            temp_.q[1] = quaternion_[1]
            temp_.q[2] = quaternion_[2]
            temp_.q[3] = quaternion_[3]
            temp_.quaternion_given = True
            temp_.roll_rate_des = 0.0
            temp_.pitch_rate_des = 0.0
            temp_.yaw_rate_des = 0.0
            temp_.thrust_des = 9.8066
            temp_.input_given = True
            trajectories_list.traj.append(temp_)
        self.trajectory_pub.publish(trajectories_list)

    def process(self):
        if self.arrive_first_waypoint:
            trajectories_list = itm_trajectory_msg()
            trajectories_list.header = Header()
            trajectories_list.traj = []
            trajectories_list.size = self.horizon + 1
            for i in range(self.horizon + 1):
                temp_ = itm_trajectory_point()
                temp_index_ = self.traj_iter + \
                    int(i * self.publish_rate * self.mpc_dt)
                if temp_index_ >= self.traj_max_iter:
                    temp_index_ = self.traj_max_iter - 1
                temp_.x = self.desired_trajectory[temp_index_, 0]
                temp_.y = self.desired_trajectory[temp_index_, 1]
                temp_.z = self.desired_trajectory[temp_index_, 2]
                temp_.vx = self.desired_trajectory[temp_index_, 6]
                temp_.vy = self.desired_trajectory[temp_index_, 7]
                temp_.vz = self.desired_trajectory[temp_index_, 8]
                temp_.roll = self.desired_trajectory[temp_index_, 3]
                temp_.pitch = self.desired_trajectory[temp_index_, 4]
                temp_.yaw = self.desired_trajectory[temp_index_, 5]
                quaternion_ = self.rpy_to_quaternion(
                    np.array([temp_.roll, temp_.pitch, temp_.yaw]))
                temp_.q[0] = quaternion_[0]
                temp_.q[1] = quaternion_[1]
                temp_.q[2] = quaternion_[2]
                temp_.q[3] = quaternion_[3]
                temp_.quaternion_given = True
                temp_.roll_rate_des = self.desired_trajectory[temp_index_, 9]
                temp_.pitch_rate_des = self.desired_trajectory[temp_index_, 10]
                temp_.yaw_rate_des = self.desired_trajectory[temp_index_, 11]
                temp_.thrust_des = self.desired_trajectory[temp_index_, 12] / 1.659
                temp_.input_given = True
                trajectories_list.traj.append(temp_)
            self.trajectory_pub.publish(trajectories_list)
            self.traj_iter += 1
        else:
            self.go_to_first_point()

    def is_ready(self):
        if self.got_robot_pose and self.got_robot_odom:
            rospy.logerr(
                "You've received both odom (sim) and pose (OptiTrack). That is impossible and unsupported! Stop and check your setup")
            return False
        elif self.got_robot_pose or self.got_robot_odom:
            return True
        else:
            return False

    def time_out(self):
        if self.time_out_counter > 1000:
            return True
        else:
            self.time_out_counter += 1
            return False

    @staticmethod
    def rpy_to_quaternion(rpy):
        '''
        @description:
            estimate the quaternion based on ZYX.
        @param {array} rqy
        @return {array} quaternion in order [w, x, y, z]
        '''
        roll_, pitch_, yaw_ = rpy
        cy = np.cos(yaw_ * 0.5)
        sy = np.sin(yaw_ * 0.5)
        cp = np.cos(pitch_ * 0.5)
        sp = np.sin(pitch_ * 0.5)
        cr = np.cos(roll_ * 0.5)
        sr = np.sin(roll_ * 0.5)

        w_ = cr * cp * cy + sr * sp * sy
        x_ = sr * cp * cy - cr * sp * sy
        y_ = cr * sp * cy + sr * cp * sy
        z_ = cr * cp * sy - sr * sp * cy
        return np.array([w_, x_, y_, z_])


if __name__ == '__main__':
    rospy.init_node('uav_squared_trajectory')

    file_path = os.path.dirname(os.path.realpath(__file__))
    load_trajectory = np.load(file_path +
                              '/saved_trajs/uav_DMOC_simplest_tracking.npy', allow_pickle=True)
    print(load_trajectory.shape)
    trajectory_gen = TrajectoryPublisher(
        horizon=21, desired_trajectory=load_trajectory)

    while not rospy.is_shutdown():
        if trajectory_gen.is_ready():
            if trajectory_gen.traj_iter <= trajectory_gen.traj_max_iter:
                trajectory_gen.process()
            else:
                trajectory_gen.stop()
                break
        else:
            if trajectory_gen.time_out():
                rospy.logerr('time out')
                break
        trajectory_gen.rate.sleep()
