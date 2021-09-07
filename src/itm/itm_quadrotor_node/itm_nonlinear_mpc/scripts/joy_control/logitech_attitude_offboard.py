#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-06-30 18:48:44
LastEditors: Wei Luo
LastEditTime: 2021-07-02 14:17:29
Note: Note
'''

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion
from itm_mav_msgs.msg import SetMission


class JoyController:
    def __init__(self, thrust_limit):
        self.joy_state = None
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.attitude_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)
        self.robot_state_pub = rospy.Publisher(
            '/itm_quadrotor_control/user_control', SetMission, queue_size=10)
        self.attitude_cmd = AttitudeTarget()
        self.thrust = 0.0
        self.roll_des = 0.0
        self.pitch_des = 0.0
        self.yaw_des = 0.0
        self.control_state = 0
        self.thrust_limit = thrust_limit

    def joy_callback(self, msg):
        # if msg.buttons[0] == 0 and msg.buttons[1] == 0 and self.control_state == 0:
        # self.thrust = min(msg.axes[1] * 0.6 + self.thrust * 0.4, 0.7)
        self.thrust = self.thrust_limit*(msg.axes[1] * 0.6 + self.thrust * 0.4)
        self.roll_des = - msg.axes[3] * 3.14159/4.
        self.pitch_des = msg.axes[4] * 3.14159/4.
        self.yaw_des = msg.axes[0] * 3.14159
        if msg.buttons[0] == 1:
            self.control_state = 2
            msg_mission = SetMission()
            msg_mission.mission_mode = 2
            self.robot_state_pub.publish(msg_mission)
        elif msg.buttons[1] == 1:
            self.control_state = 4
            msg_mission = SetMission()
            msg_mission.mission_mode = 4
            self.robot_state_pub.publish(msg_mission)
        else:
            pass

    def send_command(self):
        if self.control_state == 0:
            self.attitude_cmd.header = rospy.Header()
            self.attitude_cmd.header.stamp = rospy.Time.now()
            self.attitude_cmd.type_mask = 1+2+4
            self.attitude_cmd.thrust = self.thrust
            self.attitude_cmd.orientation = self.rpy_to_quaternion(
                self.roll_des, self.pitch_des, self.yaw_des)
            self.attitude_pub.publish(self.attitude_cmd)

    @staticmethod
    def rpy_to_quaternion(r, p, y):
        cy = np.cos(y*0.5)
        sy = np.sin(y*0.5)
        cp = np.cos(p*0.5)
        sp = np.sin(p*0.5)
        cr = np.cos(r*0.5)
        sr = np.sin(r*0.5)

        q = Quaternion()
        q.w = cr*cp*cy + sr*sp*sy
        q.x = sr*cp*cy - cr*sp*sy
        q.y = cr*sp*cy + sr*cp*sy
        q.z = cr*cp*sy - sr*sp*cy

        return q


if __name__ == "__main__":
    rospy.init_node('UAV_joy_control')
    if rospy.has_param('/logitech_joy/thrust_limit'):
        thrust_limit = rospy.get_param(
            '/logitech_joy/thrust_limit')
    else:
        thrust_limit = 0.5

    print("thrust is set to {}".format(thrust_limit))
    joy_control = JoyController(thrust_limit)
    update_rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        joy_control.send_command()

        update_rate.sleep()
