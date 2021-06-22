#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-10-02 14:33:57
LastEditors: Wei Luo
LastEditTime: 2020-11-05 16:46:13
Note: Note
'''

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from threading import Thread

class PID_controller(object):
    def __init__(self, ):
        # init the class
        self.pos = PoseStamped()
        ## create a server
        server_ = rospy.Service('uav_pid_server', SetBool, self.state_server)
        ## publisher
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)
        ## subscribe the state of the robot
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Odometry, self.subscribe_pose)


        self.pos_thread = Thread(target=self.send_command, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def subscribe_pose(self, data_pose):
        # subscribe the odometry
        self.robot_pose = data_pose

    def state_server(self, req):
        return SetBoolResponse(True, 'Hi')

    def send_command(self,):
        # it is in a single thread
        rate = rospy.Rate(50)
        self.pos.header = Header()
        self.pos.pose.position.x = 0.0
        self.pos.pose.position.y = 0.0
        self.pos.pose.position.z = 0.5
        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def PID_controller(self,):
        # TODO: implement the PID controller, the input should be the current state of the controller, and the output should be the desired thrust and angles
        pass


if __name__ == '__main__':
    rospy.init_node('offboard_pid_controller')
    uav_controller = PID_controller()
    rospy.spin()