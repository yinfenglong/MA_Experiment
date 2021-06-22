#!/usr/bin/env python

# ros package 
import rospy 

from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point 
import tf

from dynamic_reconfigure.server import Server
from itm_nonlinear_mpc.cfg import UKFObserverConfig


# general packages
import numpy as np 
from scipy.linalg import sqrtm, norm
from numpy.linalg import inv


class UKF_observer(object):
    def __init__(self, dim_x, dim_z, dt):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt
        self.imu_angular_velocity_ = np.zeros((3))
        self.imu_linear_acceleration_ = np.zeros((3))
        self.imu_orientation_ = np.zeros((4))

        # UKF parameters
        self.state = np.zeros((self.dim_x)) # state of the quadrotor [x, y, z, vx, vy, vz, q, ...]
        self.state[6] = 1.0  # since q = [w x y z] and w_init = 1.0
        self.state_prior = self.state.copy()
        self.obs_state = np.zeros((self.dim_z))
        self.P = np.diag([0.5]*self.dim_x)
        self.P_prior = self.P.copy()
        self.obs_state = np.zeros((self.dim_z))
        self.Q = np.diag([0.01, 0.02, 0.02, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.alpha_ = 0.2
        self.beta_ = 2.0 # for Gaussian priors
        self.kappa_ = 0.0
        self.lambda_ = self.alpha_**2 * (self.dim_x + self.kappa_) - self.dim_x
        self.gamma_ = np.sqrt(self.dim_x + self.lambda_)

        ## weights 
        self.W_m = np.array((2*self.dim_x+1)*[0.5/(self.dim_x + self.lambda_)])
        # print(self.W_m[0])
        # self.W_m[0] = 1.0
        # print(self.W_m)
        self.W_m[0] = self.lambda_/(self.dim_x + self.lambda_)
        self.W_c = np.array((2*self.dim_x+1)*[0.5/(self.dim_x + self.lambda_)])
        self.W_c[0] = self.W_m[0] + (1-self.alpha_**2+self.beta_)
        
        self.sigmas_f = np.zeros((2*self.dim_x+1, dim_x)) # sigma points matrix
        self.measurement_ = np.array([-1.0]*self.dim_z)

        self.acc_offset_ = np.zeros((3))
        self.angular_vel_offset_ = np.zeros((3))


        # ros subscribe topic 
        self.imu_measurement_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)
        self.pos_measurement_sub = rospy.Subscriber('/robot_pose', Odometry, self.odomCallback)
        self.estimated_pub_ = rospy.Publisher('/estimated_pub', Point, queue_size=50)

    def init_state(self,):

        idx = 5.0
        state_pos_ = [] 
        state_ori_ = [] 
        acc_off_ = []
        angul_vec_ = []
        while (idx>0):
            idx -= 1.0
            state_pos_.append(self.measurement_.copy())
            state_ori_.append(self.imu_orientation_.copy())
            acc_off_.append(self.imu_linear_acceleration_.copy())
            angul_vec_.append(self.imu_angular_velocity_.copy())
        self.state[0:3] = np.mean(state_pos_, axis=0)
        self.state[6:10] = np.mean(state_ori_, axis=0)
        self.state = self.state.reshape(self.dim_x, )
        
        self.acc_offset_ = np.mean(acc_off_, axis=0)
        # self.angular_vel_offset_ = np.mean(angul_vec_, axis=0)
        # print("acc offset: {}".format(acc_off_))
        # print("state inited as {}".format(self.state))
        # print("angular offset: {}".format(angul_vec_))
        # print("acc offset: {}".format(self.acc_offset_))
        print("state inited as {}".format(self.state))
        # print("angular offset: {}".format(self.angular_vel_offset_))


    def imuCallback(self, data):
        # angular velocities 
        self.imu_angular_velocity_[0] = data.angular_velocity.x # p
        self.imu_angular_velocity_[1] = data.angular_velocity.y # q
        self.imu_angular_velocity_[2] = data.angular_velocity.z # r

        # linear accelerations
        self.imu_linear_acceleration_[0] = data.linear_acceleration.x # a_bx 
        self.imu_linear_acceleration_[1] = data.linear_acceleration.y # a_by 
        self.imu_linear_acceleration_[2] = data.linear_acceleration.z # a_bz

        # orientation
        self.imu_orientation_[0] = data.orientation.w
        self.imu_orientation_[1] = data.orientation.x 
        self.imu_orientation_[2] = data.orientation.y
        self.imu_orientation_[3] = data.orientation.z

    def odomCallback(self, data):
         # position
        self.measurement_[0] = data.pose.pose.position.x
        self.measurement_[1] = data.pose.pose.position.y
        self.measurement_[2] = data.pose.pose.position.z
        #  # orientation
        # self.measurement_[3] = data.pose.pose.orientation.x
        # self.measurement_[4] = data.pose.pose.orientation.y
        # self.measurement_[5] = data.pose.pose.orientation.z
        # self.measurement_[6] = data.pose.pose.orientation.z

    def state_sigma_points_matrix(self, x, P):
        
        try:
            temp_matrix = sqrtm(P)
        except:
            rospy.logerr("Cannot solve the matrix square root using cholesky!")
            print(np.linalg.det(P))


        sig_matrix = np.zeros((2*self.dim_x+1, self.dim_x))
        sig_matrix[0] = x
        for i in range(self.dim_x):
            sig_matrix[i+1] = x + self.gamma_ * temp_matrix[i]
            sig_matrix[i+self.dim_x + 1] = x - self.gamma_ * temp_matrix[i]
        return sig_matrix


    def direction_cosine_matrix(self, q):
        # q is quaternion [w, x, y, z]
        # return matrix body to inertial
        if abs(norm(q) - 1.0) > 1e-2:
            q = q/norm(q)
        return 2* np.array([
            [0.5-q[2]**2-q[3]**2, q[1]*q[2]-q[0]*q[3], q[1]*q[3]+q[0]*q[2]],
            [q[1]*q[2]+q[0]*q[3], 0.5-q[1]**2-q[3]**2, q[2]*q[3]-q[0]*q[1]],
            [q[1]*q[3]-q[0]*q[2], q[2]*q[3]+q[0]*q[1], 0.5-q[1]**2-q[2]**2]
        ])


    def time_state_function(self, old_state, dt):
        new_state = np.zeros((self.dim_x))
        
        accel_temp = self.imu_linear_acceleration_ - self.acc_offset_
        
        a_temp = np.dot(self.direction_cosine_matrix(old_state[6:10]), accel_temp ) + old_state[10:13] 
        # print("acc {0}".format(a_temp))
        new_state[3:6] = old_state[3:6] + a_temp*dt

        # print("speed {0}".format(new_state[3:6]))
        
        new_state[0:3] = old_state[0:3] + new_state[3:6]*dt + 0.5 * a_temp * dt**2
        

        ## first version 
        # rotation_vector_ = (self.imu_angular_velocity_ - self.angular_vel_offset_) * dt
        # skew_matrix_ = np.array([
        #     [0.0, rotation_vector_[0], rotation_vector_[1], rotation_vector_[2]],
        #     [-rotation_vector_[0], 0.0, -rotation_vector_[2], rotation_vector_[1]],
        #     [-rotation_vector_[1], rotation_vector_[2], 0.0, -rotation_vector_[0]],
        #     [-rotation_vector_[2], -rotation_vector_[1], rotation_vector_[0], 0.0]
        # ])
        

        # s_ = norm(rotation_vector_) * 0.5
        # theta_ = 1 - np.sqrt(old_state[6]**2+old_state[7]**2+old_state[8]**2+old_state[9]**2)
        # # print("theta{}".format(theta_)) 
        # exp_matrix_ = np.diag([1.0]*4) *(np.cos(s_) + theta_*dt*1.0) - 0.5 * skew_matrix_ * np.sin(s_)/(s_+1e-6) #

        ## second version
        rotation_vel_vector_ = self.imu_angular_velocity_ - self.angular_vel_offset_ + old_state[13:16]
        skew_matrix_ = 0.5 * np.array([
            [0.0, -rotation_vel_vector_[0], -rotation_vel_vector_[1], -rotation_vel_vector_[2]],
            [rotation_vel_vector_[0], 0.0, rotation_vel_vector_[2], -rotation_vel_vector_[1]],
            [rotation_vel_vector_[1], -rotation_vel_vector_[2], 0.0, rotation_vel_vector_[0]],
            [rotation_vel_vector_[2], rotation_vel_vector_[1], -rotation_vel_vector_[0], 0.0]
        ])
        

        omega_ = np.sqrt((rotation_vel_vector_[0]*dt)**2+(rotation_vel_vector_[1]*dt)**2+(rotation_vel_vector_[2]*dt)**2) * 0.5
        
        exp_matrix_ = np.diag([1.0]*4) *(np.cos(omega_/2.0) + np.sin(omega_/2.0)*omega_*skew_matrix_)#

        new_q = np.dot(exp_matrix_, old_state[6:10])
        
        new_state[6:10] = new_q#/norm(new_q)

        new_state[10:16] = old_state[10:16]

        return new_state

    def observer_function(self, x):
        # this function indicates the relationship between the state and the observer state z
        return np.array([x[0:3]]) # x, y, z


    def time_unscented_transform(self, sigma_m, Wm, Wc):
        x = np.dot(Wm, sigma_m) # shape (self.dim_x, )
        diff_x = sigma_m - x #shape (2*self.dim_x+1, self.dim_x)
        # P = np.dot(diff_x.T, np.dot(np.diag(Wc), diff_x))
        P = np.dot(diff_x[1:].T, np.dot(np.diag(Wc[1:]), diff_x[1:]))
        return (x, P)

    def observer_unscented_transform(self, pre_x, sigma_f, sigma_h, Wm, Wc):
        y = np.dot(Wm, sigma_h)
        diff_y = sigma_h - y
        Py = np.dot(diff_y[1:].T, np.dot(np.diag(Wc[1:]), diff_y[1:]))
        diff_xy = sigma_f - pre_x
        Pxy = np.dot(diff_y.T, np.dot(np.diag(Wc), diff_xy))
        return y, Py, Pxy

    def compute_process_sigmas(self, dt):
        sigmas = self.state_sigma_points_matrix(self.state, self.P)
       
        # apply each each sigma point to the time state update function 
        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = self.time_state_function(s, dt)
            # print("sigma result {0}: \n {1}".format(i, self.sigmas_f[i]))

    def predict(self, dt=None, UT=None,):
        if dt is None:
            dt = self.dt
        if UT is None:
            UT = self.time_unscented_transform

        
        # compute the sigma points 
        self.compute_process_sigmas(dt)
        
        # compute time-update state
        self.state, self.P = UT(self.sigmas_f, self.W_m, self.W_c)

        self.state[6:10] = self.state[6:10]/norm(self.state[6:10])
        # print("state {}".format(self.state[6:10]))
        # save prior state
        self.state_prior = self.state.copy()
        self.P = self.P + self.Q
        self.P_prior = self.P.copy()
        self.sigmas_f = self.state_sigma_points_matrix(self.state, self.P)

    def update(self, z):
        # get and check the observer function
        # self.observer_function(z)

        sigmas_h = []
        for s in self.sigmas_f:
            sigmas_h.append(self.observer_function(s))
        
        self.sigmas_h = np.atleast_2d(sigmas_h).reshape(-1, self.dim_z)

        y_, Py_, Pxy_ = self.observer_unscented_transform(pre_x=self.state_prior, sigma_h=self.sigmas_h, sigma_f=self.sigmas_f, Wm=self.W_m, Wc=self.W_c)

        Py_ = Py_ + np.diag([0.01, 0.01,0.01])

        kalmanGain_ = np.dot(Pxy_.transpose(), inv(Py_))
        # print("kalman{}".format(kalmanGain_))
        self.state = self.state_prior + np.dot(kalmanGain_, z-y_)
        # print("prior {0} and \n current {1}".format(self.state_prior, self.state))
        self.P = self.P_prior - np.dot(kalmanGain_, np.dot(Py_, kalmanGain_.transpose()))
        # self.P = np.exp(self.P)
        # print("p {}".format(self.P))


if __name__ == "__main__":
    # init ros node 
    rospy.init_node('UKF_observer_py', anonymous=True)

    # model parameters
    state_size = 16 # [x v e a_b omega_b]
    obs_state_size = 3 # [x]

    # create a ukf object
    ukf_observer_obj = UKF_observer(dim_x=state_size, dim_z=obs_state_size, dt=0.033)

    # setup of the time parameters
    delta_t = 0.033
    last_time = rospy.get_time() ## get initial timer 
    rate = rospy.Rate(30)
    x_last = np.zeros((state_size)) 

    # record the continuesly error for UKF 
    error_index = 0
    success_index = 0

    # publish result 
    estimated_ = Point()

    # check if the ros/quadrotor is ready

    while not rospy.is_shutdown():
        last_time = rospy.get_time()
        if np.sum(ukf_observer_obj.imu_linear_acceleration_)>0.0: # 
            ukf_observer_obj.init_state()
            break

    print("init finish")
        

    # begin the loop
    while not rospy.is_shutdown():
        # current time 
        t_now = rospy.get_time()
        # strange error, the clock time may get zero!
        if t_now > last_time:
            delta_t = (t_now-last_time)
            ukf_observer_obj.predict(dt=delta_t)
            ukf_observer_obj.update(z=ukf_observer_obj.measurement_)
            print("state {0} \n".format(ukf_observer_obj.state))
            # estimated_.x = ukf_observer_obj.state[0]
            # estimated_.y = ukf_observer_obj.state[1]
            # estimated_.z = ukf_observer_obj.state[2]

            estimated_.x = ukf_observer_obj.state[10]
            estimated_.y = ukf_observer_obj.state[11]
            estimated_.z = ukf_observer_obj.state[12]
            ukf_observer_obj.estimated_pub_.publish(estimated_)
            last_time = rospy.get_time()
        else:
            error_index += 1
            # print("222 {}".format(t_now-last_time))
        rate.sleep()
        

        # limit the dt range, it needs to be turned 
        # if delta_t > 0.015:
        #     delta_t = 0.015
        # elif delta_t < 0.005:
        #     delta_t = 0.005
        # else:
        #     pass
        # print("dt!{}".format(delta_t))
        # init sensor 
        
        # ukf_observer_obj.predict(dt=delta_t)
        # ukf_observer_obj.update(z=ukf_observer_obj.measurement_)
        # print("index {}".format(error_index))
        # last_time = rospy.get_time()
        
        # try:
        #     ukf_observer_obj.predict(dt=delta_t)
        #     if error_index:
        #         error_index = 0
        #         print('----\n')
        #     else:
        #         success_index += 1
        #         print("success times {}.".format(success_index))
        # except:
        #     print('predict error')


