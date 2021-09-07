#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-10-21 15:01:53
LastEditors: Wei Luo
LastEditTime: 2020-11-02 14:39:52
Note: Note
'''

import rospy
import numpy as np
from itm_nonlinear_mpc.msg import itm_trajectory_msg, itm_trajectory_point
from itm_nonlinear_mpc.srv import itm_trajectory_srv
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('trajectory_command')
    rospy.wait_for_service('/itm_trajectory_srv')
    set_traj_srv = rospy.ServiceProxy('itm_trajectory_srv', itm_trajectory_srv)

    msg_index = 1
    trajectory_list = []
    point = itm_trajectory_point()
    point.x = 0.5
    point.y = 1.3
    point.z = 2.0
    point.yaw = 0.0
    point.derivative = 0
    point.fixed = True
    point.time_known = True
    point.time_stamp = 10.0
    trajectory_list.append(point)

    point = itm_trajectory_point()
    point.x = 1.8
    point.y = 1.8
    point.z = 1.5
    point.yaw = -np.pi/2.0
    point.derivative = 0
    point.time_known = True
    point.fixed = True
    point.time_stamp = 26.0
    trajectory_list.append(point)

    point = itm_trajectory_point()
    point.x = 2.6
    point.y = 0.6
    point.z = 1.0
    point.yaw = np.pi/4.
    point.derivative = 0
    point.time_known = True
    point.time_stamp = 40.0
    point.fixed = True
    trajectory_list.append(point)

    while not rospy.is_shutdown():
        srv_obj = set_traj_srv(msg_index, trajectory_list)
        if srv_obj.success:
            # only send the trajectory once
            break