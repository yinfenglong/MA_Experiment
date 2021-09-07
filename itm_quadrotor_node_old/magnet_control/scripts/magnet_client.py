#!/usr/bin/env python3
# coding=utf-8

import sys
import rospy
# from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest


def magnet_client():
    
    print("3")
    rospy.wait_for_service('/magnet_control')
    # rospy.wait_for_service('/server_magnet_control')
    controlState = rospy.get_param('~pubMagnet', False)
    try:
        mag = rospy.ServiceProxy('/magnet_control', SetBool)
        print("5")
        response = mag(controlState) 
        print("6")
        return response.success, response.message
    except:
        print ("Service call failed ")

# magnet = TriggerRequest()
# result = magnet_service(magnet)
# print (result)

if __name__ == "__main__":
    rospy.init_node('magnet_client')
    print("1")
    # print ("Reponse: %s" % (magnet_client()))
    a, b = magnet_client()
    print("2")
