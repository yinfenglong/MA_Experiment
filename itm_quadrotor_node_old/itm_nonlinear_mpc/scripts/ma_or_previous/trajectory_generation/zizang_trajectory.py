#!/usr/bin/env python
from math import pi, sin, cos

import rospy
from geometry_msgs.msg import PoseStamped
from  nav_msgs.msg  import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time

class ZTrajectory(object):
    def __init__(self, period,v, time_horizon, time_rotate, time_hovering,x=0,y=0,z=1):
        # self.side = side
        self.period = period
        self.v=v
        self.time_horizon=time_horizon
        self.time_rotate = time_rotate
        self.time_hovering=time_hovering
        # self.v = 3.0 * side / period
        self.x_0 = x
        self.y_0 = y
        self.z_0 = z
        self.position = PoseStamped()
        self.FirstPosition = np.zeros(3)
        rospy.Subscriber("/robot_pose", Odometry, self.callback, queue_size=50)
        #  rospy.Subscriber("/robot/pose", Pose, self.callback, queue_size=50)

 
    def get_position_at(self,t):
            if 0 <= t < self.time_hovering:
                self.position.pose.position.x = self.x_0
                self.position.pose.position.y = self.y_0 
                self.position.pose.position.z = self.z_0
            if self.time_hovering<= t < self.time_hovering+self.time_horizon:
                self.position.pose.position.x = self.x_0 + self.v *( t-self.time_hovering)
                self.position.pose.position.y = self.y_0 
                self.position.pose.position.z = self.z_0
            elif self.time_hovering+self.time_horizon <= t  < self.time_hovering+self.time_horizon+ self.time_rotate:
                self.position.pose.position.x = self.x_0 + self.v * self.time_horizon
                self.position.pose.position.y = self.y_0 
                self.position.pose.position.z = self.z_0
            elif self.time_hovering+self.time_horizon+ self.time_rotate <= t < self.time_hovering+ 2*self.time_horizon+ self.time_rotate:
                self.position.pose.position.x = self.x_0 + self.v * self.time_horizon-self.v*( t-(self.time_hovering+self.time_horizon+self.time_rotate))* cos(pi/4) 
                self.position.pose.position.y = self.y_0 - self.v*( t-(self.time_hovering+self.time_horizon+self.time_rotate)) * sin(pi/4) 
                self.position.pose.position.z = self.z_0
            elif self.time_hovering+ 2*self.time_horizon+ self.time_rotate <= t  < self.time_hovering+2*self.time_horizon+ 2*self.time_rotate:
                self.position.pose.position.x = self.x_0+ self.v * self.time_horizon - self.v * self.time_horizon* cos(pi/4) 
                self.position.pose.position.y = self.y_0 - self.v*self.time_horizon * sin(pi/4)
                self.position.pose.position.z = self.z_0
            elif self.time_hovering+2*self.time_horizon+ 2*self.time_rotate <= t < self.time_hovering+3*self.time_horizon+ 2*self.time_rotate:
                self.position.pose.position.x = self.x_0+ self.v * self.time_horizon - self.v * self.time_horizon* cos(pi/4) +self.v *(t-(self.time_hovering+2*self.time_horizon+ 2*self.time_rotate))
                self.position.pose.position.y = self.y_0 - self.v*self.time_horizon * sin(pi/4)
                self.position.pose.position.z = self.z_0
            elif self.time_hovering+3*self.time_horizon+ 2*self.time_rotate <= t :
                self.position.pose.position.x = self.x_0+ self.v * self.time_horizon - self.v * self.time_horizon* cos(pi/4) +self.v * self.time_horizon
                self.position.pose.position.y = self.y_0 - self.v*self.time_horizon * sin(pi/4)
                self.position.pose.position.z = self.z_0 
                
    def callback(self, data):
        self.FirstPosition[0] = data.pose.pose.position.x
        self.FirstPosition[1] = data.pose.pose.position.y
        self.FirstPosition[2] = data.pose.pose.position.z
     

if __name__ == "__main__":

    rospy.init_node('trajectory_publisher')

    trajectory_pub = rospy.Publisher("/itm_quadrotor_control/set_point_pos", PoseStamped, queue_size= 50)

    traj_obj = ZTrajectory(period=120,v=0.25, time_horizon=20, time_rotate=5, time_hovering=10, x=0, y=0, z=1)

    # if traj_obj.FirstPosition[0] - 0 > 0.1 and traj_obj.FirstPosition[1] - 0 > 0.1 and traj_obj.FirstPosition[2] -0 > 0.1:
    #     traj_obj.position.pose.position.x = traj_obj.FirstPosition[0]
    #     traj_obj.position.pose.position.y = traj_obj.FirstPosition[1]
    #     traj_obj.position.pose.position.z = traj_obj.FirstPosition[2]
    # else:
    #     traj_obj.position.pose.position.x = 0
    #     traj_obj.position.pose.position.y = 0
    #     traj_obj.position.pose.position.z = 1
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
        if t_ <= traj_obj.period:
            t_ = t_ + dt
        elif t_ > traj_obj.period:
            t_ = t_ - traj_obj.period
        else:
            print("error")
        
        traj_obj.get_position_at(t=t_)
 
        trajectory_pub.publish(traj_obj.position)
        last_time =  current_time
        # else:
            # print(t_)
        r.sleep()




