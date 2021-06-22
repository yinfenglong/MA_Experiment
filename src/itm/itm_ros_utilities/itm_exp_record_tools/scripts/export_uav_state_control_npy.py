#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-05-30 13:58:15
LastEditors: Wei Luo
LastEditTime: 2021-06-15 16:40:49
Note: Note
'''

import numpy as np

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget


class UAVSubNpy(object):
    def __init__(self, uav_pose_topic, rate_cmd=True):
        # pose/odometry
        self.uav_pose_sub = rospy.Subscriber(
            uav_pose_topic, PoseStamped, self.pose_callback)
        self.got_pose_msg = False
        self.uav_pose = None
        self.pose_timer = None
        if rate_cmd:
            # attitude rate
            self.att_rate_cmd_sub = rospy.Subscriber(
                '/mavros/setpoint_raw/attitude', AttitudeTarget, self.attitude_rate_cmd_callback)
        else:
            self.att_rate_cmd_sub = rospy.Subscriber(
                '/mavros/setpoint_raw/attitude', AttitudeTarget, self.attitude_cmd_callback)
        self.got_cmd = False
        self.att_rate_cmd = None
        self.att_cmd = None

    def pose_callback(self, msg):
        if not self.got_pose_msg:
            self.got_pose_msg = True
        quat = msg.pose.orientation
        euler_ = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        self.uav_pose = np.array([msg.pose.position.x, msg.pose.position.y,
                                 msg.pose.position.z, euler_[0], euler_[1], euler_[2]])
        self.pose_timer = rospy.get_time()

    def attitude_cmd_callback(self, msg):
        if not self.got_cmd:
            self.got_cmd = True
        r_, p_, y_ = self.quaternion_to_rpy(msg.orientation)
        self.att_cmd = np.array(
            [r_, p_, y_])
        self.cmd_timer = rospy.get_time()

    def attitude_rate_cmd_callback(self, msg):
        if not self.got_cmd:
            self.got_cmd = True
        self.att_rate_cmd = np.array(
            [msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust])
        self.cmd_timer = rospy.get_time()

    @staticmethod
    def quaternion_to_rpy(quaternion):
        q0, q1, q2, q3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        roll_ = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
        pitch_ = np.arcsin(2*(q0*q2-q3*q1))
        yaw_ = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
        return roll_, pitch_, yaw_


if __name__ == '__main__':
    rospy.init_node('uav_analyse')
    rate = rospy.Rate(100)
    is_rate_cmd = False
    sub_obj = UAVSubNpy('/vrpn_client_node/ITM_Q300/pose',
                        rate_cmd=is_rate_cmd)
    data_list = []

    if is_rate_cmd:
        while not rospy.is_shutdown():
            while sub_obj.att_rate_cmd is None or sub_obj.uav_pose is None:
                pass
            current_time_ = rospy.get_time()
            if current_time_ - sub_obj.cmd_timer > 2. or current_time_ - sub_obj.pose_timer > 2.:
                # safe the data
                np.save('exp_data.npy', data_list)
                break
            data_list.append(np.append(sub_obj.uav_pose.flatten(),
                                       sub_obj.att_rate_cmd.flatten()))
            rate.sleep()
    else:
        while not rospy.is_shutdown():
            while sub_obj.att_cmd is None or sub_obj.uav_pose is None:
                pass
            current_time_ = rospy.get_time()
            if current_time_ - sub_obj.cmd_timer > 2. or current_time_ - sub_obj.pose_timer > 2.:
                # safe the data
                np.save('exp_data.npy', data_list)
                break
            data_list.append(np.append(sub_obj.uav_pose.flatten(),
                                       sub_obj.att_cmd.flatten()))
            rate.sleep()

    rospy.loginfo('log accomplish')
