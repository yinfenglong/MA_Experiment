#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-10-06 10:03:41
LastEditors: Wei Luo
LastEditTime: 2021-03-29 13:52:55
Note: Note
'''
from numpy.testing._private.utils import print_assert_equal
import rospy
from nav_msgs.msg  import Odometry
from geometry_msgs.msg import Quaternion
from itm_nonlinear_mpc.msg import itm_trajectory_msg
import numpy as np
from mavros_msgs.msg import AttitudeTarget
from std_srvs.srv import SetBool, SetBoolResponse
from threading import Thread
from std_msgs.msg import Header
# from tf.transformations import quaternion_from_euler
import time
import acado_uav_rate

from offboard_sim_kf_observer import KF_observer

class MPC_controller(object):
    def __init__(self, horizon=20,):
        # parameters
        self.current_state = None
        self.trajectory_path = None
        self.att_command = AttitudeTarget()
        self.att_command.type_mask = 128 # ignore rate
        self.command_roll_pitch_yaw_thrust = np.array([0.0, 0.0, 0.0, 9.806])
        self.rate = rospy.Rate(100)
        self.num_uav_states = 10
        self.num_controls = 4
        ## MPC relative
        self.time_horizon = horizon
        self.mpc_opt_command = np.zeros((horizon, self.num_controls))
        self.mpc_next_states = np.zeros((horizon+1, self.num_uav_states))
        self.mpc_init = False
        self.Q = np.diag([80, 80, 120,
         50, 50, 100,
         80, 80, 80, 80,
         10.0, 10.0, 20.0, 1.0])
        self.QN = np.diag([120, 120, 120,
        50, 50, 50])
        # self.Q = np.diag([80.0, 80.0, 120.0, 80.0, 80.0, 100.0, 10.0, 10.0    , 50.0, 60.0, 1.0]) # without yaw
        # self.QN = np.diag([86.21, 86.21, 120.95, 6.94, 6.94, 11.04])
        self.next_controls = np.tile(np.array([0.0, 0.0, 0.0, 9.8066]), (self.time_horizon, 1))
        # load_parameter
        pass
        # publish
        self.att_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        # subscribers
        ## the robot state
        robot_state_sub_ = rospy.Subscriber('/robot_pose', Odometry, self.robot_state_callback)
        ## trajectory
        robot_trajectory_sub_ = rospy.Subscriber('/robot_trajectory', itm_trajectory_msg, self.trajectory_command_callback)
        # create a server
        server_ = rospy.Service('uav_mpc_server', SetBool, self.state_server)

        # offset free using KF
        self.kf_estimator = KF_observer()
        self.position_error_integration = np.zeros((1, 3)) # np.zeros((3, 1))
        self.enable_integrator = False # enable the integrator
        self.sampling_time = 0.03 # this should be the same rate of the odometry message


        # It seems that thread cannot ensure the performance of the time
        self.att_thread = Thread(target=self.send_command, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

    def mpc_estimation_loop(self,):
        if self.trajectory_path is not None and self.current_state is not None:
            # set parameters
            ## currently we do not have an assest to estimate the external force
            # ext_forces = np.tile(np.array([0.0, 0.0, 0.]), (self.time_horizon+1, 1))
            current_state_ = self.current_state
            trajectory_path_ = np.concatenate((current_state_, self.trajectory_path), axis=0)
            # mpc_state_ = np.concatenate((trajectory_path_[:-1, :], self.next_controls), axis=1)
            # print(self.next_controls.shape)
            current_yaw_ = current_state_[0, -1] # get the current yaw angle [rad]
            self.kf_estimator.feedAttitudeCommand(self.command_roll_pitch_yaw_thrust)
            self.kf_estimator.feedPositionMeasurement(current_state_[0, :3])
            self.kf_estimator.feedVelocityMeasurement(current_state_[0, 3:6])
            rmat_ = self.quad_to_rotation_matrix(current_state_[0, 6:])
            self.kf_estimator.feedRotationMeasurement(rmat_)

            is_kf_success_ = self.kf_estimator.updateEstimator()
            if not is_kf_success_:
                # self.kf_estimator.reset() # not yet finished
                # ext_forces = np.tile(np.array([0.0, 0.0, 0.]), (self.time_horizon+1, 1))
                kf_estimated_forces_ = np.zeros((1, 3))
            else:
                kf_estimated_forces_ = self.kf_estimator.state[12:15].reshape(1, -1)
                # ext_forces = np.tile(np.array([0.0, 0.0, 0.]), (self.time_horizon+1, 1))

            # if the integrator is enabled
            if self.enable_integrator:
                position_error_ = self.trajectory_path[0, :3] - current_state_[0, :3]
                if np.linalg.norm(position_error_) < 0.4:
                    self.position_error_integration += position_error_*self.sampling_time
                else:
                    self.position_error_integration = np.zeros((1, 3))
                self.position_error_integration = np.minimum(self.position_error_integration, [2, 2, 2])
                self.position_error_integration = np.maximum(self.position_error_integration, [-2, -2, -2])
                kf_estimated_forces_ = kf_estimated_forces_ - (np.array([0.2, 0.2, 0.3]) * self.position_error_integration).reshape(1, -1)

            kf_estimated_forces_body_ = np.dot(rmat_.T, kf_estimated_forces_.T)
            ext_forces = np.tile(kf_estimated_forces_, (self.time_horizon+1, 1))
            trajectory_path_[:-1, -1] = trajectory_path_[:-1, -1] - ext_forces[0, -1]
            mpc_state_ = np.concatenate((trajectory_path_[:-1, :], self.next_controls), axis=1)

            if not self.mpc_init:
                next_states_ = np.tile(current_state_, (self.time_horizon+1, 1))
                self.init_mpc = acado_uav_rate.mpc_init_function(current_state_,
                next_states_,
                self.next_controls,
                mpc_state_,
                trajectory_path_[-1, :6].reshape(1, -1),
                self.Q,
                self.QN,
                ext_forces)
                acado_uav_rate.mpc_control_bounds(
                    np.tile(np.array([-80.0/180.0*np.pi,-80.0/180.0*np.pi,-30.0/180.0*np.pi, 0.5*9.806]), (1, self.time_horizon)),
                    np.tile(np.array([80.0/180.0*np.pi, 80.0/180.0*np.pi, 30.0/180.0*np.pi, 1.5*9.806]), (1, self.time_horizon)))
                self.mpc_init = True
            else:
                X_res, U_res = acado_uav_rate.mpc(current_state_.reshape(1, -1),
                mpc_state_,
                trajectory_path_[-1, :6].reshape(1, -1),
                ext_forces)
                self.att_command.header = Header()
                self.att_command.header.stamp = rospy.Time.now()
                if np.NaN in U_res:
                    # self.att_command.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
                    self.att_command.thrust = 0.5
                    self.att_command.body_rate.z = 0.0
                    self.att_command.body_rate.x = 0.0
                    self.att_command.body_rate.y = 0.0
                else:
                    # print(U_res)
                    # self.att_command.orientation = Quaternion(*quaternion_from_euler(U_res[0, 0], U_res[0, 1], 0))
                    self.att_command.thrust = U_res[0, 3]/9.806 - 0.5
                    # print("thrust is : {}".format(self.att_command.thrust))
                    # yaw_command_ = self.yaw_command(current_yaw_, trajectory_path_[1, -1], 0.0)
                    self.att_command.body_rate.x = U_res[0, 0]
                    self.att_command.body_rate.y = U_res[0, 1]
                    self.att_command.body_rate.z = U_res[0, 2]
                    self.mpc_opt_command = np.concatenate((U_res[1:], U_res[-1:]), axis=0)
                    self.mpc_next_states = np.concatenate((current_state_, X_res[1:]), axis=0)
        else:
            if self.trajectory_path is None:
                rospy.loginfo("waiting trajectory")
            elif self.current_state is None:
                rospy.loginfo("waiting current state")
            else:
                rospy.loginfo("Unknown error")


        self.rate.sleep()

        return True

    def robot_state_callback(self, data):
        # robot state as [x, y, z, vx, vy, vz, [w, x, y, z]]
        # roll_, pitch_, yaw_ = self.quaternion_to_rpy(data.pose.pose.orientation)
        self.current_state = np.array([data.pose.pose.position.x, data.pose.pose.position.y,
        data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y,
        data.twist.twist.linear.z, data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]).reshape(1, -1)
        # print(self.current_state)
        #data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w

    @staticmethod
    def yaw_command(current, reference, rate_reference, K_yaw=1.8, max_yaw_rate=1.5):
        yaw_error_ =  - current + reference
        print('yaw error')
        print(yaw_error_)
        if np.abs(yaw_error_) > np.pi:
            if yaw_error_ > 0.0:
                yaw_error_ = yaw_error_ - 2.0*np.pi
            else:
                yaw_error_ = yaw_error_ + 2.0*np.pi
        yaw_rate_command_ = K_yaw * yaw_error_ + rate_reference
        if yaw_rate_command_ < -max_yaw_rate:
            yaw_rate_command_ = -max_yaw_rate
        elif yaw_rate_command_ > max_yaw_rate:
            yaw_rate_command_ = max_yaw_rate
        else:
            pass

        return yaw_rate_command_

    @staticmethod
    def quaternion_to_rpy(quaternion):
        q0, q1, q2, q3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        roll_ = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
        pitch_ = np.arcsin(2*(q0*q2-q3*q1))
        yaw_ = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
        return roll_, pitch_, yaw_

    @staticmethod
    def rpy_to_quaternion(r, p, y):
        cy = np.cos(y*0.5)
        sy = np.sin(y*0.5)
        cp = np.cos(p*0.5)
        sp = np.sin(p*0.5)
        cr = np.cos(r*0.5)
        sr = np.sin(r*0.5)

        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        return np.array([qw, qx, qy, qz])

    @staticmethod
    def quad_to_rotation_matrix(quad):
        # quad in [w, x, y, z]
        tx = 2.0*quad[1]
        ty = 2.0*quad[2]
        tz = 2.0*quad[3]
        twx = tx*quad[0]
        twy = ty*quad[0]
        twz = tz*quad[0]
        txx = tx*quad[1]
        txy = ty*quad[1]
        txz = tz*quad[1]
        tyy = ty*quad[2]
        tyz = tz*quad[2]
        tzz = tz*quad[3]

        res = np.array([
            [1.0-(tyy+tzz), txy-twz, txz+twy],
            [txy+twz, 1.0-(txx+tzz), tyz-twx],
            [txz-twy, tyz+twx, 1.0-(txx+tyy)]
        ])

        return res

    def trajectory_command_callback(self, data):
        temp_traj = data.traj
        if data.size != len(temp_traj):
            rospy.logerr('Some data are lost')
        else:
            self.trajectory_path = np.zeros((data.size, self.num_uav_states))
            for i in range(data.size):
                # quaternion_ = quaternion_from_euler(temp_traj[i].roll, temp_traj[i].pitch, temp_traj[i].yaw)
                quaternion_ = self.rpy_to_quaternion(temp_traj[i].roll, temp_traj[i].pitch, temp_traj[i].yaw)
                # print(quaternion_)
                # quaternion_ = [1.0, 0.0, 0.0, 0.0]
                self.trajectory_path[i] = np.array([temp_traj[i].x,
                                                    temp_traj[i].y,
                                                    temp_traj[i].z,
                                                    temp_traj[i].vx,
                                                    temp_traj[i].vy,
                                                    temp_traj[i].vz,
                                                    quaternion_[0],
                                                    quaternion_[1],
                                                    quaternion_[2],
                                                    quaternion_[3]
                ])

    def state_server(self, req):
        return SetBoolResponse(True, 'MPC is ready')

    def send_command(self,):
        rate = rospy.Rate(100)  # Hz
        self.att_command.header = Header()

        while not rospy.is_shutdown():
            t2 = time.time()
            command_ = self.att_command
            # self.att_command.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(command_)
            # command_.thrust = 5.8
            # print(command_)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
            # print("publsich loop takes {} seconds".format(time.time() - t2))

if __name__ == '__main__':
    rospy.init_node('offboard_mpc_controller')

    try:
        mpc_obj = MPC_controller()
        mpc_model_is_ready = True
    except ImportError:
        rospy.logerr('Cannot find any MPC library, Stop the node')
        mpc_model_is_ready = False
        mpc_obj = None


    while not rospy.is_shutdown() and mpc_model_is_ready:
        if not mpc_obj.mpc_estimation_loop():
            rospy.logerr("MPC estimation failed")

    print('MPC controller is shutdown')
