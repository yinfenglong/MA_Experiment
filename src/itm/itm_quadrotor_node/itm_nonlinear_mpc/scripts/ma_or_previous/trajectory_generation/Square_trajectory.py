#!/usr/bin/env python
from math import pi, sin, cos

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import numpy as np

class SquaredTrajectory(object):
    def __init__(self, side, period,x=0,y=0,z=1):
        self.side = side
        self.period = period
        self.v = 4.0 * side / period
        self.x_0 = x
        self.y_0 = y
        self.z_0 = z
        self.position = PoseStamped()
        self.FirstPosition = np.zeros(3)
        # self.FirstPosition = Pose()
        rospy.Subscriber("/hummingbird/ground_truth/pose", Pose, self.callback, queue_size=50)

 
    def get_position_at(self,t):

        if 0 <= t < self.period / 4:
            self.position.pose.position.x = self.v * t + self.x_0
            self.position.pose.position.y = self.y_0
            self.position.pose.position.z = self.z_0
        elif self.period / 4 <= t < self.period / 2:
            self.position.pose.position.x = self.x_0 + self.side
            self.position.pose.position.y = self.v * (t-self.period / 4) + self.y_0
            self.position.pose.position.z = self.z_0
        elif self.period / 2 <= t < 3 * self.period / 4:
            self.position.pose.position.x = -self.v * (t-self.period / 2) + self.x_0 + self.side
            self.position.pose.position.y = self.y_0 + self.side
            self.position.pose.position.z = self.z_0
        elif 3*self.period / 4 <= t < self.period:
            self.position.pose.position.x = self.x_0
            self.position.pose.position.y = -self.v * (t-3*self.period / 4) + self.y_0 + self.side
            self.position.pose.position.z = self.z_0
        
    
    def callback(self, data):
        self.FirstPosition[0] = data.position.x
        self.FirstPosition[1] = data.position.y
        self.FirstPosition[2] = data.position.z
     

if __name__ == "__main__":

    rospy.init_node('trajectory_publisher')

    trajectory_pub = rospy.Publisher("/hummingbird/command/pose", PoseStamped, queue_size= 50)

    traj_obj = SquaredTrajectory(side=4, period=20, x=0, y=0, z=1)

    if traj_obj.FirstPosition[0] - 0 > 0.1 and traj_obj.FirstPosition[1] - 0 > 0.1 and traj_obj.FirstPosition[2] -0 > 0.1:
        traj_obj.position.pose.position.x = traj_obj.FirstPosition[0]
        traj_obj.position.pose.position.y = traj_obj.FirstPosition[1]
        traj_obj.position.pose.position.z = traj_obj.FirstPosition[2]
    else:
        traj_obj.position.pose.position.x = 0
        traj_obj.position.pose.position.y = 0
        traj_obj.position.pose.position.z = 1

    t_ = 0
    last_time = rospy.Time.now()
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        traj_obj.get_position_at(t=t_)
        if t_ <= traj_obj.period:
            t_ = t_ + dt
        elif t_ > traj_obj.period:
            t_ = t_ - traj_obj.period
            
        trajectory_pub.publish(traj_obj.position)
        last_time =  current_time
        r.sleep()




