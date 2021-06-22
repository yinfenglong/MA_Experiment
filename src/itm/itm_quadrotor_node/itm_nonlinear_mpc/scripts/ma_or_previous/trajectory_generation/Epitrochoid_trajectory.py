#!/usr/bin/env python
from math import pi, sin, cos

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import numpy as np

class EpitrochoidTrajectory(object):
    def __init__(self, R, r, d, period, scale_factor = 1, x = 0, y = 0, z =1):
        self.R = R
        self.r = r
        self.d = d
        self.period = period
        self.scale_factor = scale_factor
        self.x_0 = x
        self.y_0 = y
        self.z_0 = z
        self.position = PoseStamped()
        self.FirstPosition = np.zeros(3)
        # self.FirstPosition = Pose()
        #  rospy.Subscriber("/mavros/local_position/pose", Pose, self.callback, queue_size=50)
        rospy.Subscriber("/robot_pose", Pose, self.callback, queue_size=50)

    
    def get_position_at(self,t):
        self.position.pose.position.x = self.scale_factor * ((self.R + self.r) * cos(2*pi*t/self.period)-self.d*cos(((self.R + self.r)/self.r)*2*pi*t/self.period))
        self.position.pose.position.y = self.scale_factor * ((self.R + self.r) * sin(2*pi*t/self.period)-self.d*sin(((self.R + self.r)/self.r)*2*pi*t/self.period))
        self.position.pose.position.z = self.z_0
    
    def callback(self, data):
        self.FirstPosition[0] = data.position.x
        self.FirstPosition[1] = data.position.y
        self.FirstPosition[2] = data.position.z
     

if __name__ == "__main__":

    rospy.init_node('trajectory_publisher')

    trajectory_pub = rospy.Publisher("/itm_quadrotor_control/set_point_pos", PoseStamped, queue_size= 50)

    traj_obj = EpitrochoidTrajectory(R=4, r= 1, d = 1, period=2*pi, scale_factor = 1, x = 0, y = 0, z =1)

    if traj_obj.FirstPosition[0] - 4 > 0.1 and traj_obj.FirstPosition[1] - 0 > 0.1 and traj_obj.FirstPosition[2] -1 > 0.1:
        traj_obj.position.pose.position.x = traj_obj.FirstPosition[0]
        traj_obj.position.pose.position.y = traj_obj.FirstPosition[1]
        traj_obj.position.pose.position.z = traj_obj.FirstPosition[2]
    else:
        traj_obj.position.pose.position.x = 4
        traj_obj.position.pose.position.y = 0
        traj_obj.position.pose.position.z = 1

    t_ = 0

    last_time = rospy.Time.now()
    r = rospy.Rate(60.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        traj_obj.get_position_at(t=t_)
        t_ = t_ + dt
        trajectory_pub.publish(traj_obj.position)
        last_time =  current_time
        r.sleep()
    





