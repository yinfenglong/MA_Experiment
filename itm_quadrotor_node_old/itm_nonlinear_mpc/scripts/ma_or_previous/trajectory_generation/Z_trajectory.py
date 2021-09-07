#!/usr/bin/env python
from math import pi, sin, cos

import rospy
from geometry_msgs.msg import PoseStamped
from  nav_msgs.msg  import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time

class ZTrajectory(object):
    def __init__(self, side, period,x=0,y=0,z=1):
        self.side = side
        self.period = period
        self.v=0.25
        # self.v = 3.0 * side / period
        self.x_0 = x
        self.y_0 = y
        self.z_0 = z
        self.position = PoseStamped()
        self.FirstPosition = np.zeros(3)
        # self.FirstPosition = Pose()
        rospy.Subscriber("/robot_pose", Odometry, self.callback, queue_size=50)
        #  rospy.Subscriber("/robot/pose", Pose, self.callback, queue_size=50)

 
    def get_position_at(self,t,n):
            if 0 <= t < self.period / 6:
                self.position.pose.position.x = self.x_0 + self.v * t
                self.position.pose.position.y = self.y_0 
                self.position.pose.position.z = self.z_0
            elif self.period / 6 <= t < self.period/3:
                # self.position.pose.position.x = self.x_0 + self.v * self.period / 6 - self.v * (t-self.period/6) * cos(pi/3)
                self.position.pose.position.x = (self.x_0 + self.v*self.period/6) - self.v * (t-self.period/6) * cos(pi/4)
                self.position.pose.position.y = self.y_0 - self.v * (t-self.period/6)* sin(pi/4)
                self.position.pose.position.z = self.z_0  
            elif self.period /3 <= t < self.period/2:
                # self.position.pose.position.x = self.x_0 + self.v * self.period / 6 *(1-cos(pi/3)) + self.v * (t - self.period/3)
                self.position.pose.position.x = (self.x_0 + self.v*self.period/6*(1-cos(pi/4))) + self.v * (t - self.period/3)
                self.position.pose.position.y = self.y_0 - self.v*self.period/6 * sin(pi/4) 
                self.position.pose.position.z = self.z_0 
            elif self.period /2 <= t < 2*self.period/3:
                self.position.pose.position.x = self.x_0 + self.v*self.period/6 *(2-cos(pi/4)) - self.v * (t - self.period/2)
                self.position.pose.position.y = self.y_0 - self.v*self.period/6 * sin(pi/4)
                self.position.pose.position.z = self.z_0
            elif 2*self.period /3 <= t < 5*self.period/6:
                self.position.pose.position.x = self.x_0 + self.v*self.period/6 *(1-cos(pi/4)) + self.v * (t-2*self.period/3) * cos(pi/4)
                self.position.pose.position.y = self.y_0 - self.v*self.period/6 * sin(pi/4) + self.v * (t-2*self.period/3) * sin(pi/4)
                self.position.pose.position.z = self.z_0 
            elif 5*self.period /6 <= t < self.period:
                self.position.pose.position.x = self.x_0 + self.v*self.period/6 - self.v * (t-5*self.period/6)
                self.position.pose.position.y = self.y_0 
                self.position.pose.position.z = self.z_0 
                
    def callback(self, data):
        self.FirstPosition[0] = data.pose.pose.position.x
        self.FirstPosition[1] = data.pose.pose.position.y
        self.FirstPosition[2] = data.pose.pose.position.z
     

if __name__ == "__main__":

    rospy.init_node('trajectory_publisher')

    trajectory_pub = rospy.Publisher("/itm_quadrotor_control/set_point_pos", PoseStamped, queue_size= 50)

    traj_obj = ZTrajectory(side=5.0, period=120.0, x=0, y=0, z=1)

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
    n_ = 0
    
    while not rospy.Time.now():
        pass
    last_time = rospy.Time.now()
    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        # if not rospy.Time.now():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            if t_ <= traj_obj.period:
                t_ = t_ + dt
            elif t_ > traj_obj.period:
                t_ = t_ - traj_obj.period
            else:
                print("error")
        
            traj_obj.get_position_at(t=t_, n= n_)
 
            trajectory_pub.publish(traj_obj.position)
            last_time =  current_time
        # else:
            # print(t_)
        
            r.sleep()




