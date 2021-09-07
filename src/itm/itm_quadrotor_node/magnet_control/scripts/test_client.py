#!/usr/bin/env python
# coding=utf-8

import sys
import rospy
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('test_client')

rospy.wait_for_service('/test_control')

test_service = rospy.ServiceProxy('/test_control', Trigger)

test = TriggerRequest()

result = test_service(test)

print result


