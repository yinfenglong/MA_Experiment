#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-10-06 10:03:41
LastEditors: Wei Luo
LastEditTime: 2021-03-11 10:15:28
Note: Note
'''
import rospy
from nav_msgs.msg  import Odometry
from geometry_msgs.msg import Quaternion
from itm_nonlinear_mpc.msg import itm_trajectory_msg
import numpy as np
import casadi as ca
from mavros_msgs.msg import AttitudeTarget
from std_srvs.srv import SetBool, SetBoolResponse
from threading import Thread
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import time

class MPC_controller(object):
    def __init__(self, lib_path=None, mpc_c=None, horizon=20, solver_=None):
        # parameters
        self.current_state = None
        self.trajectory_path = None
        self.att_command = AttitudeTarget()
        self.att_command.type_mask = 3 # ignore rate
        self.rate = rospy.Rate(100)
        ## MPC relative
        self.time_horizon = horizon
        self.mpc_opt_command = np.zeros((horizon, 3))
        self.mpc_next_states = np.zeros((horizon+1, 9))
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
        # load MPC solver
        ## ipopt option
        ipopt_options = {
            'verbose': False,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-4,
            "ipopt.max_iter": 100,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.print_level": 0,
            "print_time": False
        }
        if lib_path is not None: # directly load the library
            self.solver = ca.nlpsol('solver', 'ipopt', lib_path, ipopt_options)
        if mpc_c is not None: # another alternative to compile and load the library
            import_c_so = ca.Importer(mpc_c, 'clang')
            self.solver = ca.nlpsol('solver', 'ipopt', import_c_so, ipopt_options)

        if lib_path is None and mpc_c is None:
            if solver_ is not None:
                self.solver = solver_.solver
            else:
                rospy.logerr("No solver!!!")

        # It seems that thread cannot ensure the performance of the time
        self.att_thread = Thread(target=self.send_command, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

    def mpc_estimation_loop(self,):
        if self.trajectory_path is not None and self.current_state is not None:
            # currently we do not have an assest to estimate the external force
            ext_forces = np.array([0.0, 0.0, 0.]).reshape(-1, 1)
            # define the constraints
            lbg = 0.0
            ubg = 0.0
            lbx = []
            ubx = []
            for _ in range(self.time_horizon):
                lbx = lbx + [np.deg2rad(-45), np.deg2rad(-45), 0.5*9.8066]
                ubx = ubx + [np.deg2rad(45), np.deg2rad(45), 1.5*9.8066]
            for _ in range(self.time_horizon+1):
                lbx = lbx + [-np.inf]*9
                ubx = ubx + [np.inf]*9
            # t1 = time.time()
            # set parameters
            current_state_ = self.current_state
            trajectory_path_ = np.concatenate((current_state_, self.trajectory_path), axis=0)
            # print(trajectory_path_[:4])
            control_params = ca.vertcat(ext_forces.reshape(-1, 1), trajectory_path_.reshape(-1, 1))
            # print('control_p {}'.format(trajectory_path_.shape))
            init_control = ca.vertcat(self.mpc_opt_command.reshape(-1, 1), self.mpc_next_states.reshape(-1, 1))

            # print("the reference trajectory {}".format(self.trajectory_path))
            # print("control params: {0} \n and the init_control: {1}".format(control_params, init_control))
            sol = self.solver(x0=init_control, p=control_params, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)
            ## get results
            estimated_opt = sol['x'].full()
            mpc_u_ = estimated_opt[:int(3*(self.time_horizon))].reshape(self.time_horizon, 3)
            mpc_x_ = estimated_opt[int(3*(self.time_horizon)):].reshape(self.time_horizon+1, 9)
            # print("estimated command: {0} \n and the estimated state: {1}".format(mpc_u_[:4], mpc_x_[:4]))
            self.att_command.orientation = Quaternion(*quaternion_from_euler(mpc_u_[0, 0], mpc_u_[0, 1], 0))
            self.att_command.thrust = mpc_u_[0, 2]/9.806 - 0.5
            # publish the data directly
            self.att_command.header = Header()
            self.att_command.header.stamp = rospy.Time.now()
            # self.att_setpoint_pub.publish(self.att_command)
            # print(self.att_command.thrust)
            self.mpc_opt_command = np.concatenate((mpc_u_[1:], mpc_u_[-1:]), axis=0)
            # self.mpc_next_states = np.tile(self.current_state, (self.time_horizon, 1))
            self.mpc_next_states = np.concatenate((current_state_, mpc_x_[1:]), axis=0)
            # print("loop time {}".format(time.time()-t1))
            # self.mpc_next_states = self.current_state
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
        # robot state as [x, y, z, vx, vy, vz, [x, y, z, w]]
        roll_, pitch_, yaw_ = self.quaternion_to_rpy(data.pose.pose.orientation)
        self.current_state = np.array([data.pose.pose.position.x, data.pose.pose.position.y,
        data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y,
        data.twist.twist.linear.z, roll_, pitch_, yaw_]).reshape(1, -1)
        # print(self.current_state)
        #data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w

    @staticmethod
    def quaternion_to_rpy(quaternion):
        q0, q1, q2, q3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        roll_ = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
        pitch_ = np.arcsin(2*(q0*q2-q3*q1))
        yaw_ = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
        return roll_, pitch_, yaw_

    def trajectory_command_callback(self, data):
        temp_traj = data.traj
        if data.size != len(temp_traj):
            rospy.logerr('Some data are lost')
        else:
            self.trajectory_path = np.zeros((data.size, 9))
            for i in range(data.size):
                self.trajectory_path[i] = np.array([temp_traj[i].x,
                                                    temp_traj[i].y,
                                                    temp_traj[i].z,
                                                    temp_traj[i].vx,
                                                    temp_traj[i].vy,
                                                    temp_traj[i].vz,
                                                    temp_traj[i].roll,
                                                    temp_traj[i].pitch,
                                                    temp_traj[i].yaw
                ])

    def state_server(self, req):
        return SetBoolResponse(True, 'MPC is ready')

    def send_command(self,):
        rate = rospy.Rate(100)  # Hz

        while not rospy.is_shutdown():
            # t2 = time.time()
            command_ = self.att_command
            # self.att_command.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(command_)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
            # print("publsich loop takes {} seconds".format(time.time() - t2))

if __name__ == '__main__':
    rospy.init_node('offboard_mpc_controller')

    if rospy.has_param('offboard_mpc_controller/lib_path'):
        casadi_lib = rospy.get_param('offboard_mpc_controller/lib_path')
        mpc_obj = MPC_controller(lib_path=casadi_lib)
        mpc_model_is_ready = True
    elif rospy.has_param('offboard_mpc_controller/mpc_c_file'):
        casadi_c = rospy.get_param('offboard_mpc_controller/mpc_c_file')
        mpc_obj = MPC_controller(mpc_c_file=casadi_c)
        mpc_model_is_ready = True
    else:
        try:
            from non_ros_casadi import casadi_sx_ros as ca_ros# no test yet
            solver_ = ca_ros.MPC_UAV(dt=0.1, N=20)
            mpc_obj = MPC_controller(solver_=solver_)
            mpc_model_is_ready = True
        except ImportError:
            rospy.logerr('Cannot find any MPC library, Stop the node')
            mpc_model_is_ready = False
            mpc_obj = None


    while not rospy.is_shutdown() and mpc_model_is_ready:
        if not mpc_obj.mpc_estimation_loop():
            rospy.logerr("MPC estimation failed")

    print('MPC controller is shutdown')
