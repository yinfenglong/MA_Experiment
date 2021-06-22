#!/usr/bin/env python2

import rospy
import mavros
import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, ParamValue
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, WaypointClear, WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange


class mavros_core_function():
    def __init__(self, uav_number=None):

        mavros.set_namespace()

        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.standard_pose = PoseStamped(Header(1, rospy.Time.now(), ""), Pose(Point(1, 2, 3), Quaternion(0, 0, 0, 0)))
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.delta_error = 0.08
        if uav_number is None:
            self.uav_number_str = '0'


        # Set all topics to not ready (e.g. 'alt': False)
        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }

        # ROS services

        # Waiting for ROS services to come up
        service_timeout = 30

        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException as e:
                print(e)

        # Setup ROS services
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)

        # Setup ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        # self.ext_state_sub = rospy.Subscriber('mavros/extended_state', ExtendedState, self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu, self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        # self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)

        # # Setup ROS Publisher
        # if self.uav_number is None:
        #     self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        # else:
        #     self.local_pos_pub = rospy.Publisher('/uav' + self.uav_number_str + '/' + "mavros/setpoint_position/local",
        #                                          PoseStamped, queue_size=10)


    def tearDown(self):
        self.log_topic_vars()

    #
    # Callback functions
    #
    def altitude_callback(self, data):
        self.altitude = data

        # A1607MSL (Above mean sea level) has been observed to be nan while other fields are valid
        # Set topic 'alt' as ready
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    # def extended_state_callback(self, data):
    #     # Print UAV state change
    #     if self.extended_state.vtol_state != data.vtol_state:
    #         rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "VTOL state changed from {0} to {1}".format(
    #             mavutil.mavlink.enums['MAV_VTOL_STATE'][self.extended_state.vtol_state].name,
    #             mavutil.mavlink.enums['MAV_VTOL_STATE'][data.vtol_state].name))

    #     # Print landed state change
    #     if self.extended_state.landed_state != data.landed_state:
    #         rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "landed state changed from {0} to {1}".format(
    #             mavutil.mavlink.enums['MAV_LANDED_STATE'][self.extended_state.landed_state].name,
    #             mavutil.mavlink.enums['MAV_LANDED_STATE'][data.landed_state].name))

    #     self.extended_state = data

    #     # Set topic 'ext_state' as ready
    #     if not self.sub_topics_ready['ext_state']:
    #         self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data):
        self.global_position = data

        # Set topic 'global_pos' as ready
        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data):
        self.imu_data = data

        # Set topic 'imu' as ready
        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data):
        self.home_position = data

        # Set topic 'home_pos' as ready
        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data

        # Set topic 'local_pos' as ready
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    # def mission_wp_callback(self, data):
    #     # Update waypoints
    #     if self.mission_wp.current_seq != data.current_seq:
    #         rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "current mission waypoint sequence updated: {0}"
    #                       .format(data.current_seq))

    #     self.mission_wp = data

    #     # Set topic 'mission_wp' as ready
    #     if not self.sub_topics_ready['mission_wp']:
    #         self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        # Print armed state change
        if self.state.armed != data.armed:
            rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "armed state changed from {0} to {1}"
                          .format(self.state.armed, data.armed))

        # Print connection state change
        if self.state.connected != data.connected:
            rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "connected changed from {0} to {1}"
                          .format(self.state.connected, data.connected))

        # Print mode state change
        if self.state.mode != data.mode:
            rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "mode changed from {0} to {1}"
                          .format(self.state.mode, data.mode))

        # Print system status change
        if self.state.system_status != data.system_status:
            rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][self.state.system_status].name,
                mavutil.mavlink.enums['MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        # Set topic 'state' as ready
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    #
    # Helper methods
    #

    def set_mavros_parameter(self, param_id, value):
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)

        try:
            param_response = self.set_param_srv(param_id, val)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        if param_response.success:
            rospy.loginfo("Changing parameter " + str(param_id) + " to " + str(value) + " was successful.")
        else:
            rospy.loginfo("Changing parameter " + str(param_id) + " to " + str(value) + " failed.")


    # def is_position_reached(self, position):
    #     if (-self.delta_error <= self.local_position.pose.position.x - position.pose.position.x <= self.delta_error and
    #         -self.delta_error <= self.local_position.pose.position.y - position.pose.position.y <= self.delta_error and
    #         -self.delta_error <= self.local_position.pose.position.z - position.pose.position.z <= self.delta_error):
    #         return True
    #     else:
    #         return False

    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)

        for i in xrange(timeout * loop_freq):

            # Try to arm UAV
            if self.state.armed == arm:
                rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "set arm success | seconds: {0} of {1}"
                              .format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr('UAV ' + self.uav_number_str + ': ' + "failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)


    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo('UAV ' + self.uav_number_str + ': ' "setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):

            # Try to set mode
            if self.state.mode == mode:
                rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "set mode success | seconds: {0} of {1}"
                              .format(i / loop_freq, timeout))
                break
            else:
                try:
                    # if mode == "OFFBOARD":
                    #     rospy.loginfo('UAV ' + self.uav_number_str + ': ' +
                    #                   "Sending setpoints before starting OFFBOARD mode.")
                    #     for i in range(10):
                    #         self.local_pos_pub.publish(self.standard_pose)
                    #         rospy.Rate(20).sleep()

                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr('UAV ' + self.uav_number_str + ': ' + "failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)


    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False

        for i in xrange(timeout * loop_freq):

            # Check if all topics are ready
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "simulation topics ready | seconds: {0} of {1}"
                              .format(i / loop_freq, timeout))
                break

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # Add failed log


    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][desired_landed_state].name, index))

        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False

        for i in xrange(timeout * loop_freq):

            # Check for landed state
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo('UAV ' + self.uav_number_str + ': ' + "landed state confirmed | seconds: {0} of {1}"
                              .format(i / loop_freq, timeout))
                break

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # Print landed state not the same
        self.assertTrue(landed_state_confirmed, (
            "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][desired_landed_state].name,
                   mavutil.mavlink.enums['MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                   index, timeout)))


    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo("waiting for VTOL transition | transition: {0}, index: {1}".
              format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name, index))

        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False

        for i in xrange(timeout * loop_freq):

            # Check for VTOL state
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1}".format(i / loop_freq, timeout))
                transitioned = True
                break

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # Print no transition detected
        self.assertTrue(transitioned,
                       ("transition not detected | desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".
                        format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
                               mavutil.mavlink.enums['MAV_VTOL_STATE'][self.extended_state.vtol_state].name,
                               index, timeout)))


    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared = False

        for i in xrange(timeout * loop_freq):

            # Try to clear waypoints
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo("clear waypoints success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # Print failed to clear waypoints
        self.assertTrue(wps_cleared, ("failed to clear waypoints | timeout(seconds): {0}".format(timeout)))


    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False

        for i in xrange(timeout * loop_freq):

            # Try to send waypoints
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo("number of waypoints transferred: {0}".
                                  format(len(waypoints)))
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # Print sending waypoints failed
        self.assertTrue((wps_sent and wps_verified),
                        "mission could not be transferred and verified | timeout(seconds): {0}".format(timeout))


    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False

        for i in xrange(timeout * loop_freq):

            # Check for MAV_TYPE (What type of robot e.g. Quadcopter)
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo("MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type].name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            # Try to sleep
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # Print failed to get MAV_TYPE
        self.assertTrue(res.success, ("MAV_TYPE param get failed | timeout(seconds): {0}".format(timeout)))


    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")
