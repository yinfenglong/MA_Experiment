#!/usr/bin/env python
from math import pi, sin, cos

import rospy
from geometry_msgs.msg import PoseStamped
from  nav_msgs.msg  import Odometry
from geometry_msgs.msg import Pose
import numpy as np

class CircularTrajectory(object):
    def __init__(self, period, time_hovering,  scale, x = 0, y = 0, z =1 ):
        self.period = period
        self.time_hovering = time_hovering
        self.scale=scale
        self.x_0 = x
        self.y_0 = y
        self.z_0 = z
        self.position = PoseStamped()
        self.FirstPosition = np.zeros(3)
        rospy.Subscriber("/robot_pose", Odometry, self.callback, queue_size=50)

    def get_position_at(self,t):
        if 0 <= t < self.time_hovering:
            self.position.pose.position.x = self.x_0
            self.position.pose.position.y = self.y_0 
            self.position.pose.position.z = self.z_0
        else:
            self.position.pose.position.x =self.scale * sin((t- self.time_hovering)/self.period) + self.x_0
            self.position.pose.position.y =self.scale *  cos((t- self.time_hovering)/self.period)*sin((t- self.time_hovering)/self.period)+ self.y_0
            self.position.pose.position.z = self.z_0
    
    def callback(self, data):
        self.FirstPosition[0] = data.pose.pose.position.x
        self.FirstPosition[1] = data.pose.pose.position.y
        self.FirstPosition[2] = data.pose.pose.position.z
     

if __name__ == "__main__":

    rospy.init_node('trajectory_publisher')

    trajectory_pub = rospy.Publisher("/itm_quadrotor_control/set_point_pos", PoseStamped, queue_size= 50)

    traj_obj = CircularTrajectory(period=10,time_hovering=10,scale=5, x=0, y=0, z=1)

    # if traj_obj.FirstPosition[0] - 0 > 0.1 and traj_obj.FirstPosition[1] - 5 > 0.1 and traj_obj.FirstPosition[2] -0 > 0.1:
    #     traj_obj.position.pose.position.x = traj_obj.FirstPosition[0]
    #     traj_obj.position.pose.position.y = traj_obj.FirstPosition[1]
    #     traj_obj.position.pose.position.z = traj_obj.FirstPosition[2]
    # else:
    traj_obj.position.pose.position.x = 0
    traj_obj.position.pose.position.y = 0
    traj_obj.position.pose.position.z = 1

    t_ = 0
    while not rospy.Time.now():
        pass
    last_time = rospy.Time.now()
    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        traj_obj.get_position_at(t=t_)
        t_ = t_ + dt
        trajectory_pub.publish(traj_obj.position)
        last_time =  current_time
        r.sleep()
    





