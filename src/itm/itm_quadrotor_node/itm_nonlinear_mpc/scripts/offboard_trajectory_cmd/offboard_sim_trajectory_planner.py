#!/usr/bin/env python3

'''
Author: Wei Luo
Date: 2020-10-07 10:57:58
LastEditors: Wei Luo
LastEditTime: 2021-08-27 13:28:13
Note: Publish the estimated polynomial-based trajectories for the UAV
'''
import rospy
import rospkg
import os
import sys
from itm_mav_msgs.msg import itm_trajectory_msg, itm_trajectory_point
from itm_mav_srvs.srv import itm_trajectory_srv, itm_trajectory_srvResponse
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

# rospack = rospkg.RosPack()
# pack_path = rospack.get_path('itm_trajectory_planner')
# sys.path.append(pack_path + '/scripts')
# from poly_based_trajectory_planner import uav_poly_trajectory as poly_traj
import sys
import os.path
sys.path.append( os.path.join(os.path.join(os.path.dirname(__file__), '..')))
from traj_gen import uav_poly_trajectory as poly_traj

import numpy as np


class TrajectoryGenerator(object):
    def __init__(self, steps=20, init_position=np.array([0.0, 0.0, 0.8]), method='poly', index=0):
        # parameters
        # self.trajectories = itm_trajectory_msg()
        # self.trajectories.size = int(steps)
        self.rate = rospy.Rate(30)
        self.time_horizon = steps
        self.frequency = 1 / 30.  # 50Hz
        self.init_position = init_position
        self.is_arrive_init = False
        # publishers
        # estimated trajectory
        self.trajectory_pub = rospy.Publisher(
            '/robot_trajectory', itm_trajectory_msg, queue_size=50)
        self.publish_index = -1
        self.previous_trajectory = None
        self.trajectory_estimated = None
        # subscribers
        # read the current state of the robot
        self.current_state = np.array(
            [0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # None
        self.robot_state_sub = rospy.Subscriber(
            '/robot_pose', Odometry, self.robot_state_callback)
        # server
        self.trajectory_index = -1
        self.trajectory_srv = rospy.Service(
            'itm_trajectory_srv', itm_trajectory_srv, self.trajectory_server_callback)
        # trajectory method
        self.trajectory_method = method
        # rospy.Subscriber('/trajectory_keyframe', itm_trajectory_msg, self.trajectory_nodes_callback)

    def traj_publish_process(self,):
        trajectories_list = itm_trajectory_msg()
        trajectories_list.header = Header()
        trajectories_list.traj = []
        trajectories_list.size = self.time_horizon

        if self.is_arrive_init:
            rospy.loginfo_once('init position arrived')
            if self.trajectory_estimated is not None:
                if self.publish_index < self.trajectory_index:
                    # new trajectory comes
                    self.publish_index = self.trajectory_index
                    # trajectories_list = itm_trajectory_msg()
                    # trajectories_list.header = Header()
                    self.previous_trajectory = self.trajectory_estimated.T.copy()
                else:
                    # not new message
                    pass

            if self.previous_trajectory is not None:

                if self.trajectory_method == 'poly':
                    for i in range(self.time_horizon):
                        temp_ = itm_trajectory_point()
                        temp_.x = self.previous_trajectory[i, 0]
                        temp_.y = self.previous_trajectory[i, 1]
                        temp_.z = self.previous_trajectory[i, 2]
                        temp_.vx = self.previous_trajectory[i, 3]
                        temp_.vy = self.previous_trajectory[i, 4]
                        temp_.vz = self.previous_trajectory[i, 5]
                        # temp_.yaw = self.previous_trajectory[i, 6]
                        temp_.yaw = self.previous_trajectory[i, -1]
                        temp_.roll = 0.0
                        temp_.pitch = 0.0
                        quaternion_ = self.rpy_to_quaternion(
                            np.array([0.0, 0.0, temp_.yaw]))
                        temp_.q[0] = quaternion_[0]
                        temp_.q[1] = quaternion_[1]
                        temp_.q[2] = quaternion_[2]
                        temp_.q[3] = quaternion_[3]
                        # print(temp_.q)
                        temp_.quaternion_given = True
                        trajectories_list.traj.append(temp_)
                    self.trajectory_pub.publish(trajectories_list)

                    if self.previous_trajectory.shape[0] > self.time_horizon - 1:
                        # delete first way point
                        # print(self.previous_trajectory)
                        # print(self.previous_trajectory.shape)
                        self.previous_trajectory = np.concatenate(
                            (self.previous_trajectory[1:], self.previous_trajectory[-1, :].reshape(1, -1)))
                elif self.trajectory_method == 'fixed':
                    # print(self.previous_trajectory)
                    temp_ = itm_trajectory_point()
                    temp_.x = self.previous_trajectory[0, 0]
                    temp_.y = self.previous_trajectory[0, 1]
                    temp_.z = self.previous_trajectory[0, 2]
                    temp_.vx = self.previous_trajectory[0, 3]
                    temp_.vy = self.previous_trajectory[0, 4]
                    temp_.vz = self.previous_trajectory[0, 5]
                    temp_.roll = self.previous_trajectory[0, 6]
                    temp_.pitch = self.previous_trajectory[0, 7]
                    temp_.yaw = self.previous_trajectory[0, 8]
                    quaternion_ = self.rpy_to_quaternion(
                        np.array([temp_.roll, temp_.pitch, temp_.yaw]))
                    temp_.q[0] = quaternion_[0]
                    temp_.q[1] = quaternion_[1]
                    temp_.q[2] = quaternion_[2]
                    temp_.q[3] = quaternion_[3]
                    temp_.quaternion_given = True
                    trajectories_list.traj.append(temp_)
                    self.trajectory_pub.publish(trajectories_list)

        else:
            a = itm_trajectory_point()
            a.z = self.init_position[2]
            a.y = self.init_position[1]
            a.x = self.init_position[0]
            a.roll = 0.0
            a.pitch = 0.0
            a.yaw = 0.0
            trajectories_list.traj = [a] * self.time_horizon

            self.trajectory_pub.publish(trajectories_list)
            distance = np.linalg.norm(
                self.current_state[:3] - self.init_position)
            if distance < 0.4:
                self.is_arrive_init = True
            else:
                print(distance)
        self.rate.sleep()

    def trajectory_server_callback(self, req):
        if self.trajectory_index < req.index and self.current_state is not None:
            self.trajectory_index = req.index
            if self.trajectory_method == 'poly':
                pin_list = []
                time_list = []
                # read the current state
                current_state_ = self.current_state.copy()
                pin_ = {'t': 0.0, 'd': 0, 'X': np.array(
                    [current_state_[0], current_state_[1], current_state_[2], current_state_[-1]])}
                time_list.append(0.0)
                pin_list.append(pin_)
                # pin_ = {'t':0.0, 'd':1, 'X':np.array([current_state_[3], current_state_[4], current_state_[5], 0.0])}
                # pin_list.append(pin_)
                for i in req.traj:
                    if i.time_known:
                        if i.fixed:
                            pin_ = {'t': i.time_stamp, 'd': i.derivative,
                                    'X': np.array([i.x, i.y, i.z, i.yaw])}
                            if i.derivative == 0:
                                time_list.append(i.time_stamp)
                        else:  # cube
                            pass
                    else:
                        time_list.append(None)
                    pin_list.append(pin_)
                # if None in time_list:
                #     time_knots = np.array(time_list)
                # else:
                #     time_knots = np.sort(np.array(time_list))
                time_knots = np.array(time_list)
                self.pTraj = poly_traj.UAVTrajGen(
                    time_knots, [8, 4], maxContiOrder_=[4, 2])

                for j in pin_list:
                    self.pTraj.addPin(j)

                self.pTraj.setDerivativeObj(
                    np.array([0.0, 0.0, 0.0, 1.0]), np.array([0.0, 1.0]))
                self.pTraj.solve()
                if self.pTraj.isSolved:
                    # the result
                    ts = np.linspace(self.pTraj.Ts[0], self.pTraj.Ts[-1], int(
                        (self.pTraj.Ts[-1] - self.pTraj.Ts[0]) / self.frequency))
                    Xs = self.pTraj.eval(ts, 0)
                    vs = self.pTraj.eval(ts, 1)
                    self.trajectory_estimated = np.concatenate(
                        (Xs, vs), axis=0)
                    return itm_trajectory_srvResponse(True)
                else:
                    return itm_trajectory_srvResponse(False)
            elif self.trajectory_method == 'fixed':
                self.trajectory_estimated = np.array([
                    req.traj[0].x,
                    req.traj[0].y,
                    req.traj[0].z,
                    req.traj[0].vx,
                    req.traj[0].vy,
                    req.traj[0].vz,
                    req.traj[0].roll,
                    req.traj[0].pitch,
                    req.traj[0].yaw,
                ]).reshape(-1, 1)
                return itm_trajectory_srvResponse(True)

            else:
                pass
        else:
            if self.current_state is None:
                rospy.loginfo("No current state avaliable")
            else:
                rospy.loginfo("Please check the index of the trajectory")
            return itm_trajectory_srvResponse(False)

    def robot_state_callback(self, data):
        # robot state as [x, y, z, vx, vy, vz, [x, y, z, w]]
        roll_, pitch_, yaw_ = self.quaternion_to_rpy(
            data.pose.pose.orientation)
        self.current_state = np.array([data.pose.pose.position.x, data.pose.pose.position.y,
                                       data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y,
                                       data.twist.twist.linear.z, roll_, pitch_, yaw_])

    @staticmethod
    def quaternion_to_rpy(quaternion):
        q0, q1, q2, q3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        roll_ = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
        pitch_ = np.arcsin(2 * (q0 * q2 - q3 * q1))
        yaw_ = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
        return roll_, pitch_, yaw_

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
    rospy.init_node('trajectory_publish_node')
    if rospy.has_param('/offboard_sim_trajectory_gen/time_horizon'):
        time_horizon = rospy.get_param(
            '/offboard_sim_trajectory_gen/time_horizon')
    else:
        time_horizon = 20

    if rospy.has_param('/offboard_sim_trajectory_gen/trajectory_type_index'):
        traj_type_idx = rospy.get_param(
            '/offboard_sim_trajectory_gen/trajectory_type_index')
    else:
        rospy.loginfo('no trajectory type set, choose polynomial as default')
        traj_type_idx = 0

    if traj_type_idx == 0:
        traj_gen_method = 'poly'
        rospy.loginfo("set polynomial trajectory")
    elif traj_type_idx == 1:
        traj_gen_method = 'fixed'
        time_horizon = 1
        rospy.loginfo("set fixed point trajectory")
    else:
        rospy.logerr("No generator for trajectory defined!")

    trajectory_gen = TrajectoryGenerator(init_position=np.array([0, 0, 0.5]),
                                         steps=time_horizon, method=traj_gen_method)  # index=trajectory_index,
    while not rospy.is_shutdown():
        trajectory_gen.traj_publish_process()
