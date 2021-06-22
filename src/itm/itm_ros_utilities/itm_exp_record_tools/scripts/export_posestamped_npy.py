#!/usr/bin/env python2
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-05-03 10:13:05
LastEditors: Wei Luo
LastEditTime: 2021-06-07 17:30:17
Note: Note
'''

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped


class SubPoseNpy(object):
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.pose_sub_ = rospy.Subscriber(
            self.topic_name, PoseStamped, self.pose_callback)
        self.robot_pose = None
        self.pose_timer = None

    def pose_callback(self, msg):
        quat = msg.pose.orientation
        euler_ = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        roll_, pitch_, yaw_ = euler_
        self.robot_pose = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll_, pitch_, yaw_])
        self.pose_timer = rospy.get_time()


if __name__ == '__main__':
    rospy.init_node('export_pose_npy')
    topic_name = rospy.get_param('~topic_name')
    sample_rate = rospy.get_param('~sample_rate')

    rate = rospy.Rate(sample_rate)

    sub_pose_obj = SubPoseNpy(topic_name)
    pose_list = []
    while not rospy.is_shutdown():
        while sub_pose_obj.robot_pose is None:
            pass
        current_time_ = rospy.get_time()
        if current_time_ - sub_pose_obj.pose_timer > 2.:
            np.save('pose_vector_'+str(sample_rate)+'Hz.npy', pose_list)
            break
        pose_list.append(sub_pose_obj.robot_pose)
        rate.sleep()
    rospy.loginfo('Log info accomplish')
