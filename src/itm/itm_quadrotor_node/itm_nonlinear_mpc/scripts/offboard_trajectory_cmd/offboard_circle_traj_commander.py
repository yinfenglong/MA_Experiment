#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-07-31 20:42:56
LastEditors: Wei Luo
LastEditTime: 2021-08-15 23:20:59
Note: Note
'''

import rospy
import numpy as np
from itm_mav_msgs.msg import itm_trajectory_msg, itm_trajectory_point
from itm_mav_srvs.srv import itm_trajectory_srv
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


class TrajectoryGenerator(object):
    def __init__(self, horizon=20, traj_period=120, publish_rate=100, mpc_dt=0.1):
        self.robot_pose = np.array([])
        self.got_robot_pose = False
        self.got_robot_odom = False
        self.robot_odom_sub_ = rospy.Subscriber(
            '/robot_pose', Odometry, self.robot_odom_callback)
        self.robot_pose_sub_ = rospy.Subscriber(
            '/vrpn_client_node/ITM_Q330/pose', PoseStamped, self.robot_pose_callback)
        self.trajectory_pub = rospy.Publisher(
            '/robot_trajectory', itm_trajectory_msg, queue_size=50)
        self.publish_rate = publish_rate
        self.rate = rospy.Rate(publish_rate)
        self.time_out_counter = 0
        self.arrive_first_waypoint = False
        self.horizon = horizon
        self.traj_iter = 0
        self.traj_period = traj_period
        self.mpc_dt = mpc_dt  # mpc defined dt in the MPC code

        self.scale = np.pi * 2.0 / self.traj_period / self.publish_rate

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

    def go_to_first_point(self):
        pose_error = np.array([0.5, 0.0, 0.5]).flatten() - self.robot_pose
        if (np.linalg.norm(pose_error) < 0.1):
            self.arrive_first_waypoint = True
            rospy.sleep(3)
        else:
            trajectories_list = itm_trajectory_msg()
            trajectories_list.header = Header()
            trajectories_list.traj = []
            trajectories_list.size = self.horizon
            for i in range(self.horizon):
                temp_ = itm_trajectory_point()
                temp_.x = 0.5
                temp_.y = 0.0
                temp_.z = 0.5
                temp_.vx = 0.0
                temp_.vy = 0.0
                temp_.vz = 0.0
                temp_.yaw = 0.0
                temp_.roll = 0.0
                temp_.pitch = 0.0
                quaternion_ = self.rpy_to_quaternion(
                    np.array([0.0, 0.0, temp_.yaw]))
                temp_.q[0] = quaternion_[0]
                temp_.q[1] = quaternion_[1]
                temp_.q[2] = quaternion_[2]
                temp_.q[3] = quaternion_[3]
                temp_.quaternion_given = True
                trajectories_list.traj.append(temp_)
            self.trajectory_pub.publish(trajectories_list)

    def stop(self):
        trajectories_list = itm_trajectory_msg()
        trajectories_list.header = Header()
        trajectories_list.traj = []
        trajectories_list.size = self.horizon
        for i in range(self.horizon):
            temp_ = itm_trajectory_point()
            temp_.x = 0.0
            temp_.y = 0.0
            temp_.z = 0.5
            temp_.vx = 0.0
            temp_.vy = 0.0
            temp_.vz = 0.0
            temp_.yaw = 0.0
            temp_.roll = 0.0
            temp_.pitch = 0.0
            quaternion_ = self.rpy_to_quaternion(
                np.array([0.0, 0.0, temp_.yaw]))
            temp_.q[0] = quaternion_[0]
            temp_.q[1] = quaternion_[1]
            temp_.q[2] = quaternion_[2]
            temp_.q[3] = quaternion_[3]
            temp_.quaternion_given = True
            trajectories_list.traj.append(temp_)
        self.trajectory_pub.publish(trajectories_list)

    def process(self):
        if self.arrive_first_waypoint:
            trajectories_list = itm_trajectory_msg()
            trajectories_list.header = Header()
            trajectories_list.traj = []
            trajectories_list.size = self.horizon
            for i in range(self.horizon):
                temp_ = itm_trajectory_point()
                # temp_.x = 0.5 * \
                #     np.cos((self.traj_iter + i) /
                #            self.traj_period * 2.0 * np.pi)
                # temp_.y = 0.5 * \
                #     np.sin((self.traj_iter + i) /
                #            self.traj_period * 2.0 * np.pi)
                # temp_.z = 0.5
                # temp_.vx = -0.5 / self.traj_period * 2.0 * np.pi * \
                #     np.sin((self.traj_iter + i) /
                #            self.traj_period * 2.0 * np.pi)
                # temp_.vy = 0.5 / self.traj_period * 2.0 * np.pi * np.cos((self.traj_iter + i) /
                #                                                          self.traj_period * 2.0 * np.pi)
                temp_.x = 0.5 * \
                    np.cos(self.scale * (self.traj_iter + i *
                           self.publish_rate * self.mpc_dt))
                temp_.y = 0.5 * \
                    np.sin(self.scale * (self.traj_iter + i *
                           self.publish_rate * self.mpc_dt))
                temp_.z = 0.5
                temp_.vx = -0.5 * self.scale *\
                    np.sin(self.scale * (self.traj_iter + i *
                           self.publish_rate * self.mpc_dt))
                temp_.vy = 0.5 * self.scale * \
                    np.cos(self.scale * (self.traj_iter + i *
                           self.publish_rate * self.mpc_dt))
                temp_.vz = 0.0
                temp_.yaw = 0.0
                temp_.roll = 0.0
                temp_.pitch = 0.0
                quaternion_ = self.rpy_to_quaternion(
                    np.array([0.0, 0.0, temp_.yaw]))
                temp_.q[0] = quaternion_[0]
                temp_.q[1] = quaternion_[1]
                temp_.q[2] = quaternion_[2]
                temp_.q[3] = quaternion_[3]
                temp_.quaternion_given = True
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

    trajectory_gen = TrajectoryGenerator(traj_period=40, horizon=21,)

    while not rospy.is_shutdown():
        if trajectory_gen.is_ready():
            if trajectory_gen.traj_iter <= trajectory_gen.traj_period * trajectory_gen.publish_rate:
                trajectory_gen.process()
            else:
                trajectory_gen.stop()
                break
        else:
            if trajectory_gen.time_out():
                rospy.logerr('time out')
                break
        trajectory_gen.rate.sleep()
