#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-10-07 10:57:58
LastEditors: Wei Luo
LastEditTime: 2020-11-03 17:31:24
Note: Publish the estimated trajectories to the robot
'''
import rospy

from itm_nonlinear_mpc.msg import itm_trajectory_msg, itm_trajectory_point
from itm_nonlinear_mpc.srv import itm_trajectory_srv, itm_trajectory_srvResponse
from std_msgs.msg import Header
from nav_msgs.msg  import Odometry
from traj_gen import uav_poly_trajectory as poly_traj

import numpy as np

class TrajectoryGenerator(object):
    def __init__(self, steps=20, init_position=np.array([0.0, 0.0, 0.8])):
        # parameters
        # self.trajectories = itm_trajectory_msg()
        # self.trajectories.size = int(steps)
        self.rate = rospy.Rate(30)
        self.time_horizon = steps
        self.frequency = 1/30. # 50Hz
        self.init_position = init_position
        self.is_arrive_init = False
        # publishers
        ## estimated trajectory
        self.trajectory_pub = rospy.Publisher('/robot_trajectory', itm_trajectory_msg, queue_size=50)
        self.publish_index = -1
        self.previous_trajectory = None
        self.trajectory_estimated = None
        # subscribers
        ## read the current state of the robot
        self.current_state = np.array([0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])#None
        self.robot_state_sub = rospy.Subscriber('/robot_pose', Odometry, self.robot_state_callback)
        # server
        self.trajectory_index = -1
        self.trajectory_srv = rospy.Service('itm_trajectory_srv', itm_trajectory_srv, self.trajectory_server_callback)
        # rospy.Subscriber('/trajectory_keyframe', itm_trajectory_msg, self.trajectory_nodes_callback)

    def traj_publish_process(self,):
        trajectories_list = itm_trajectory_msg()
        trajectories_list.header = Header()
        trajectories_list.traj = []
        trajectories_list.size = self.time_horizon

        print(self.trajectory_estimated)

        if self.is_arrive_init:
            rospy.loginfo_once('init position arrived')
            if self.trajectory_estimated is not None:
                if self.publish_index < self.trajectory_index:
                    # new trajectory comes
                    self.publish_index = self.trajectory_index
                    # trajectories_list = itm_trajectory_msg()
                    # trajectories_list.header = Header()
                    self.previous_trajectory = self.trajectory_estimated.T.copy()
                    print(self.previous_trajectory.shape)
                else:
                    # not new message
                    pass

            if self.previous_trajectory is not None:
                for i in range(self.time_horizon):
                    temp_ = itm_trajectory_point()
                    temp_.x = self.previous_trajectory[i, 0]
                    temp_.y = self.previous_trajectory[i, 1]
                    temp_.z = self.previous_trajectory[i, 2]
                    temp_.vx = self.previous_trajectory[i, 3]
                    temp_.vy = self.previous_trajectory[i, 4]
                    temp_.vz = self.previous_trajectory[i, 5]
                    temp_.yaw = self.previous_trajectory[i, 6]
                    trajectories_list.traj.append(temp_)
                self.trajectory_pub.publish(trajectories_list)
                print(trajectories_list)

                if self.previous_trajectory.shape[0] > self.time_horizon-1:
                    # delete first way point
                    self.previous_trajectory = np.concatenate((self.previous_trajectory[1:], self.previous_trajectory[-1, :].reshape(1, -1)))

        else:
            a = itm_trajectory_point()
            a.z = self.init_position[2]
            a.y = self.init_position[1]
            a.x = self.init_position[0]
            trajectories_list.traj = [a]*self.time_horizon

            self.trajectory_pub.publish(trajectories_list)
            distance = np.linalg.norm(self.current_state[:3] - self.init_position)
            if distance < 0.3:
                self.is_arrive_init = True
            else:
                print(distance)
        self.rate.sleep()

    def trajectory_server_callback(self, req):
        if self.trajectory_index < req.index and self.current_state is not None:
            self.trajectory_index = req.index
            data_len = len(req.traj)
            pin_list = []
            time_list = []
            # read the current state
            current_state_ = self.current_state.copy()
            pin_ = {'t':0.0, 'd':0, 'X':np.array([current_state_[0], current_state_[1], current_state_[2], current_state_[-1]])}
            time_list.append(0.0)
            pin_list.append(pin_)
            pin_ = {'t':0.0, 'd':1, 'X':np.array([current_state_[3], current_state_[4], current_state_[5], 0.0])}
            pin_list.append(pin_)
            for i in req.traj:
                if i.time_known:
                    if i.fixed:
                        pin_ = {'t':i.time_stamp, 'd':i.derivative, 'X':np.array([i.x, i.y, i.z, i.yaw])}
                        if i.derivative == 0:
                            time_list.append(i.time_stamp)
                    else: # cube
                        pass
                else:
                    time_list.append(None)
                pin_list.append(pin_)
            # if None in time_list:
            #     time_knots = np.array(time_list)
            # else:
            #     time_knots = np.sort(np.array(time_list))
            time_knots = np.array(time_list)
            # print(time_knots)
            self.pTraj = poly_traj.UAVTrajGen(time_knots, [8, 4], maxContiOrder_=[4, 2])

            # print(pin_list)
            for j in pin_list:
                self.pTraj.addPin(j)

            self.pTraj.setDerivativeObj(np.array([0.0, 0.0, 0.0, 1.0]), np.array([0.0, 1.0]))
            self.pTraj.solve()
            if self.pTraj.isSolved:
                # the result
                ts = np.linspace(self.pTraj.Ts[0], self.pTraj.Ts[-1], int((self.pTraj.Ts[-1]-self.pTraj.Ts[0])/self.frequency))
                Xs = self.pTraj.eval(ts, 0)
                vs = self.pTraj.eval(ts, 1)
                self.trajectory_estimated = np.concatenate((Xs, vs), axis=0)
                return itm_trajectory_srvResponse(True)
            else:
                return itm_trajectory_srvResponse(False)
        else:
            if self.current_state is None:
                rospy.loginfo("No current state avaliable")
            else:
                rospy.loginfo("Please check the index of the trajectory")
            return itm_trajectory_srvResponse(False)

    def robot_state_callback(self, data):
        # robot state as [x, y, z, vx, vy, vz, [x, y, z, w]]
        roll_, pitch_, yaw_ = self.quaternion_to_rpy(data.pose.pose.orientation)
        self.current_state = np.array([data.pose.pose.position.x, data.pose.pose.position.y,
        data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y,
        data.twist.twist.linear.z, roll_, pitch_, yaw_])

    @staticmethod
    def quaternion_to_rpy(quaternion):
        q0, q1, q2, q3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        roll_ = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
        pitch_ = np.arcsin(2*(q0*q2-q3*q1))
        yaw_ = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
        return roll_, pitch_, yaw_


if __name__ == '__main__':
    rospy.init_node('trajectory_publish_node')
    if rospy.has_param('/offboard_sim_trajectory_gen/time_horizon'):
        time_horizon = rospy.get_param('/offboard_sim_trajectory_gen/time_horizon')
    else:
        time_horizon = 20

    trajectory_gen = TrajectoryGenerator(steps=time_horizon)
    while not rospy.is_shutdown():
        trajectory_gen.traj_publish_process()

