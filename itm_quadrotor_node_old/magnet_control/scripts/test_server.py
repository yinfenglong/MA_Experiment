#!/usr/bin/env python
# coding=utf-8
import thread, time
import rospy
import sys 
from std_srvs.srv import Trigger, TriggerResponse

pubTest = False

def test_thread():
    while True:
        if pubTest:
            print ("test start!")
        else:
            print ("test stop")

        time.sleep(0.1)

def testCallback(req):
    global pubTest
    pubTest = bool(1-pubTest)

    rospy.loginfo("Publish Test![%d]", pubTest)

    return TriggerResponse(1, "Test!")

def test_server():
    rospy.init_node('test_server')
    s = rospy.Service('/test_control', Trigger, testCallback)
    print("Ready to receive test comman.")
    thread.start_new_thread(test_thread, ())
    rospy.spin()

if __name__ == "__main__":
    test_server()
