#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-10-25 20:15:58
LastEditors: Wei Luo
LastEditTime: 2020-11-03 12:30:43
Note: Note
'''

import rospy
from geometry_msgs.msg import Pose
import numpy as np
import time
import scipy.linalg

class KF_observer(object):
    def __init__(self, state_size=18, measurement_size=9, sampling_time=0.033):
        self.state_size = state_size
        self.measurement_size = measurement_size
        self.state_covariance = np.zeros((state_size, state_size))
        self.process_noise_covariance = np.zeros((state_size, 1))
        self.measurement_covariance = np.zeros((measurement_size, 1))
        self.state = np.zeros((state_size, 1))
        self.predicted_state = np.zeros((state_size, 1))
        self.measurement = np.zeros((measurement_size, 1))
        self.forces_offset = np.zeros((3, 1))
        self.moments_offset = np.zeros((3, 1))
        self.initial_state_covariance = np.zeros((state_size, 1))

        self.roll_damping = 0.95
        self.roll_omega = 7.07
        self.roll_gain = 0.8
        self.pitch_damping = 0.76
        self.pitch_omega = 7.07
        self.pitch_gain = 0.84
        self.yaw_damping = 0.95
        self.yaw_omega = 5.0
        self.yaw_gain = 1.0

        # timer
        self.t_previous = None

        self.KF_publisher = rospy.Publisher('/itm_quadrotor/KF', Pose, queue_size=50)
        self.initialized = False
        self.initialize(sampling_time)

    def initialize(self, sampling_time):
        rospy.loginfo('start initializing KF observer')
        # continuos time model
        F_continuos_time = np.zeros((self.state_size, self.state_size))
        F_continuos_time[0, 3] = 1.0
        F_continuos_time[1, 4] = 1.0
        F_continuos_time[2, 5] = 1.0
        F_continuos_time[3, 3] = -1.0 * 0.01
        F_continuos_time[4, 4] = -1.0 * 0.01
        F_continuos_time[5, 5] = -1.0 * 0.0
        F_continuos_time[3, 12] = 1.0
        F_continuos_time[4, 13] = 1.0
        F_continuos_time[5, 14] = 1.0
        F_continuos_time[6, 9] = 1.0
        F_continuos_time[7, 10] = 1.0
        F_continuos_time[8, 11] = 1.0
        F_continuos_time[9, 6] = -1.0 * self.roll_omega*self.roll_omega
        F_continuos_time[10, 7] = -1.0 * self.pitch_omega*self.pitch_omega
        F_continuos_time[11, 8] = -1.0 * self.yaw_omega*self.yaw_omega
        F_continuos_time[9, 9] = -2.0 * self.roll_omega*self.roll_damping
        F_continuos_time[10, 10] = -2.0 * self.pitch_omega*self.pitch_damping
        F_continuos_time[11, 11] = -2.0 * self.yaw_omega*self.yaw_damping
        F_continuos_time[9, 15] = 1.0
        F_continuos_time[10, 16] = 1.0
        F_continuos_time[11, 17] = 1.0

        self.F_ = scipy.linalg.expm(sampling_time * F_continuos_time)

        print(self.F_)

        rospy.loginfo("KF parameters set up.")

        self.H_ = np.zeros((self.measurement_size, self.state_size)) #np.diag([1.0]*self.measurement_size)
        for i in range(self.measurement_size):
            self.H_[i, i] = 1.0

        self.initial_state_covariance = np.array(
            [0.1, 0.1, 0.1, # p0 position
            0.1, 0.1, 0.1,  # p0 velocity
            0.1, 0.1, 0.1,  # p0 attitude
            0.1, 0.1, 0.1,  # p0 angular velocity
            0.1, 0.1, 0.1,  # p0 force
            0.1, 0.1, 0.1,  # p0 torque
        ])
        self.state_covariance = np.diag(self.initial_state_covariance)

        self.process_noise_covariance = np.array([
            0.01, 0.01, 0.01, # q position
            0.025, 0.025, 0.025, # q velocity
            0.015, 0.015, 0.015, # q attitude
            0.02, 0.02, 0.02, # q angular velocity
            0.1, 0.1, 0.1, # q force
            0.1, 0.1, 0.1  # q torque
        ])

        self.measurement_covariance = np.array([
            0.001, 0.001, 0.001, # r position
            0.0012, 0.0012, 0.0012, # r velocity
            0.01, 0.01, 0.01, # r attitude
        ])

        self.external_forces_limit = np.array([5.0, 5.0, 3.0])
        self.external_momentum_limit = np.array([20.0, 20.0, 20.0])
        self.omega_limit = np.array([3.0, 3.0, 2.0])
        self.drag_coefficients_matrix = np.diag([0.1, 0.1, 0.0])

        self.initialized = True
        rospy.loginfo('KF observer initialized')

    def startCalibration(self):
        if self.initialized:
            self.is_calibrated = True
            self.forces_offset = np.zeros((3, 1))
            self.moments_offset = np.zeros((3, 1))
            self.calibration_counter = 0
            self.start_calibration_time = rospy.time.now
            return True
        else:
            return False

    def feedPositionMeasurement(self, input):
        self.measurement[0:3] = input.reshape(-1, 1)

    def feedVelocityMeasurement(self, input):
        self.measurement[3:6] = input.reshape(-1, 1)

    def feedRotationMeasurement(self, input):
        self.rotation_matrix = input.copy()
        self.measurement[6] = np.arctan2(input[2, 1], input[2, 2])
        self.measurement[7] = np.arcsin(input[2, 0])
        self.measurement[8] = np.arctan2(input[1, 0], input[0, 0])

    def feedAttitudeCommand(self, command):
        self.roll_pitch_yaw_thrust_cmd = command


    def updateEstimator(self,):
        if not self.initialized:
            return False

        if self.t_previous is None:
            dt = 0.033
            self.t_previous = time.time()
        else:
            dt = time.time() - self.t_previous

        if dt > 0.05:
            dt = 0.05
        elif dt < 0.016:
            dt = 0.016
        else:
            pass

        self.state_covariance = np.dot(np.dot(self.F_, self.state_covariance), self.F_.T)

        self.state_covariance = self.state_covariance + np.diag(self.process_noise_covariance)

        # predict state
        self.systemDynamics(dt)

        tmp_ = np.dot(np.dot(self.H_, self.state_covariance), self.H_.T) + np.diag(self.measurement_covariance)

        K_ = np.dot(np.dot(self.state_covariance, self.H_.T), np.linalg.inv(tmp_))

        # update with measurements
        self.state = self.predicted_state + np.dot(K_, self.measurement - np.dot(self.H_, self.state))

        # update convariance
        self.state_covariance = np.dot(np.diag([1.0]*self.state_size) - np.dot(K_, self.H_), self.state_covariance)

        omega_ = self.state[9:12]
        external_forces_ = self.state[12:15]
        external_moments_ = self.state[15:]

        omega_ = np.maximum(omega_, [-3., -3., -2.])
        omega_ = np.minimum(omega_, [3., 3., 2.])

        external_forces_ = np.maximum(external_forces_.reshape(1, -1), [-5., -5., -3])
        external_forces_ = np.minimum(external_forces_, [5., 5., 3]).reshape(-1, 1)

        kf_pub_obj_ = Pose()
        kf_pub_obj_.position.x = external_forces_[0]
        kf_pub_obj_.position.y = external_forces_[1]
        kf_pub_obj_.position.z = external_forces_[2]

        self.KF_publisher.publish(kf_pub_obj_)

        external_moments_ = np.maximum(external_moments_.reshape(1, -1), [-20., -20., -20.])
        external_moments_ = np.minimum(external_moments_, [20., 20., 20.]).reshape(-1, 1)

        self.state[12:15] = external_forces_
        self.state[15:] = external_moments_

        return True



    def systemDynamics(self, dt):
        old_state_ = self.state.copy()
        thrust_ = np.array([0.0, 0.0, self.roll_pitch_yaw_thrust_cmd[3]]).reshape(-1, 1) # change shape to 3x1

        acceleration_ = np.dot(self.rotation_matrix, thrust_) - np.array([0.0, 0.0, 9.8066]).reshape(-1, 1) + np.dot(self.drag_coefficients_matrix, old_state_[3:6]) + old_state_[12:15]

        new_velocity_ = old_state_[3:6] + acceleration_ * dt
        new_position_ = old_state_[:3] + old_state_[3:6] * dt + 0.5*acceleration_*dt**2

        angular_acceleration = np.array([
            -2.0 * self.roll_damping * self.roll_omega * old_state_[9] - self.roll_omega * self.roll_omega * old_state_[6] + self.roll_gain * self.roll_omega * self.roll_omega * self.roll_pitch_yaw_thrust_cmd[0] + old_state_[15],
            -2.0 * self.pitch_damping * self.pitch_omega * old_state_[10] - self.pitch_omega * self.pitch_omega * old_state_[7] + self.pitch_gain * self.pitch_omega * self.pitch_omega * self.roll_pitch_yaw_thrust_cmd[1] + old_state_[16],
            -2.0 * self.yaw_damping * self.yaw_omega * old_state_[11] - self.yaw_omega * self.yaw_omega * old_state_[8] + self.yaw_gain * self.yaw_omega * self.yaw_omega * self.roll_pitch_yaw_thrust_cmd[2] + old_state_[17]
        ]).reshape(-1, 1)

        new_omega_ = old_state_[9:12] + angular_acceleration * dt
        new_attitude_ = old_state_[6:9] + old_state_[9:12] * dt + 0.5 * angular_acceleration * dt**2

        new_external_forces_ = old_state_[12:15]
        new_external_moments_ = old_state_[15:]

        self.predicted_state[:3] = new_position_
        self.predicted_state[3:6] = new_velocity_
        self.predicted_state[6:9] = new_attitude_
        self.predicted_state[9:12] = new_omega_
        self.predicted_state[12:15] = new_external_forces_
        self.predicted_state[15:18] = new_external_moments_


if __name__ == '__main__':
    rospy.init_node('KF_observer')
