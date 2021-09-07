#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
import rostest


if __name__ == '__main__':
    
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun('px4', 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)