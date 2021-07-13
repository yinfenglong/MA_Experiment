#!/usr/bin/env python3
# coding=utf-8

import sys
import rospy
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('magnet_client')

rospy.wait_for_service('/magnet_control')

magnet_service = rospy.ServiceProxy('/magnet_control', Trigger)

magnet = TriggerRequest()

result = magnet_service(magnet)

print (result)


