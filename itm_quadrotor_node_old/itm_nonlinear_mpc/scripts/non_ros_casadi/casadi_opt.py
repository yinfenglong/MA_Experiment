#/usr/bin/env python

import casadi as ca
import casadi.tools as ca_tools
import numpy as np
import time
from os import system

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MPC_opt(object):
    def __init__(self, state_dim, N=20, dt=0.33):
        # some parameters
        self.Ts = dt # a qusestion: should I put it as a parameter
        self.horizon = N # time horizon to predict
        self.opti = ca.Opti()
        self.g_ = 9.8066
        self.trajectory = None
        # self.trajectory = np.array(
        #         [[0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #          [0.0, 0.0, 0.4, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.67, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.69, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.73, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.83, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.85, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.88, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.91, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.93, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.95, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.97, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #         ])
        # model variables
        self.num_control = 3
        self.controls = self.opti.variable(self.horizon, self.num_control) # control variables in shape (N-1, 3) where [roll_soll, pitch_soll thrust_soll]
        self.roll_soll = self.controls[:, 0]
        self.pitch_soll = self.controls[:, 1]
        self.thrust_soll = self.controls[:, 2]
        ## control relevant parameters
        self.roll_tau = 0.257
        self.roll_gain = 0.75
        self.pitch_tau = 0.259
        self.pitch_gain = 0.78

        ## model states
        self.states = self.opti.variable(self.horizon+1, state_dim)
        self.Q_m = self.opti.parameter(8, 8) # position, velocity, roll, pitch (not yaw included
        self.P_m = self.opti.parameter(6, 6) # end tern k
        self.R_m = self.opti.parameter(3, 3) # control
        temp_P_ = np.diag([ 86.21, 86.21, 120.95, 6.94, 6.94, 11.04]) # only p and v
        temp_P_[0, 3] = 6.45
        temp_P_[3, 0] = 6.45
        temp_P_[1, 4] = 6.45
        temp_P_[4, 1] = 6.45
        temp_P_[2, 5] = 10.95
        temp_P_[5, 2] = 10.95
        temp_Q_ = np.diag([80.0, 80.0, 120.0, 80.0, 80.0,100.0, 10.0, 10.0])
        temp_R_ = np.diag([50.0, 60.0, 1.0])
        ### setup the penalty matrixes
        self.opti.set_value(self.Q_m, temp_Q_)
        self.opti.set_value(self.P_m, temp_P_)
        self.opti.set_value(self.R_m, temp_R_)
        ### reference trajectory of the quadrotor
        self.X_ref = self.opti.parameter(self.horizon+1, state_dim)
        ### additional forces
        self.external_forces = self.opti.parameter(3) # external force x, y, z on body frame
        # objective function definition
        self.obj = self.objective_function()

        # define the object function
        self.opti.minimize(self.obj)
        # constrains
        ## initial condition
        self.opti.subject_to(self.states[0, :]==self.X_ref[0, :])
        ## dynamic continue constrains
        for i in range(N-1):
            # k1 = self.dyn_function(self.states[i, :], self.controls[i, :])
            # k2 = self.dyn_function(self.states[i, :]+self.Ts/2.0*k1, self.controls[i, :])
            # k3 = self.dyn_function(self.states[i, :]+self.Ts/2.0*k2, self.controls[i, :])
            # k4 = self.dyn_function(self.states[i, :]+self.Ts*k3, self.controls[i, :])
            # x_next = self.states[i, :] + self.Ts/6.0*(k1+2.0*k2+2.0*k3+k4)
            # x_next = k1*self.Ts + self.states[i, :]
            x_next_  = self.RK_4(self.states[i, :], self.controls[i, :], self.external_forces)
            # define the constraints
            self.opti.subject_to(self.states[i+1, :]==x_next_)
        ## control constrains
        self.opti.subject_to(self.opti.bounded(np.radians(-45), self.roll_soll, np.radians(45)))
        self.opti.subject_to(self.opti.bounded(np.radians(-45), self.pitch_soll, np.radians(45)))
        self.opti.subject_to(self.opti.bounded(0.5*self.g_, self.thrust_soll, 1.5*self.g_))
        self.opti.subject_to(self.opti.bounded(-np.pi/2.0, self.states[:, 6], np.pi/2.0))

        ## opti solver
        opts_setting = {'ipopt.max_iter':200,'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
        self.opti.solver('ipopt', opts_setting)

        # func_export = self.opti.to_function('quad', [self.external_forces, self.X_ref], [self.states, self.controls])
        # cname = func_export.generate()
        # oname = 'quad.so'
        # system('gcc -fPIC -shared ' + cname + ' -o ' + oname)
        ###### following is only for test
        ## set the trajectory
        # self.opti.set_value(self.X_ref,  self.trajectory)
        # # self.opti.set_value(self.X_0, np.array(([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))

        # self.solver = self.opti.solve()
        # print(self.solver.value(self.states))
        # print(self.solver.value(self.controls))

    def aero_drag(self, states, controls):
        # aero dynamic drag acceleration
        linear_drag_coefficient = [0.01, 0.01]
        thrust = controls[2]
        v = states[3:6]
        euler = states[6:]
        dragacc1 = np.sin(euler[1])*thrust*v[2] + np.cos(euler[1])*np.cos(euler[2])*linear_drag_coefficient[0]*thrust*v[0] - np.cos(euler[1])*linear_drag_coefficient[1]*np.sin(euler[2])*thrust*v[1]
        dragacc2 = (np.cos(euler[0])*np.sin(euler[2])-np.cos(euler[2])*np.sin(euler[0])*np.sin(euler[1]))*linear_drag_coefficient[1]*thrust*v[0] - (np.cos(euler[0])*np.cos(euler[2]) + np.sin(euler[1])*np.sin(euler[0])*np.sin(euler[2]))*linear_drag_coefficient[1]*thrust*v[1] - np.cos(euler[1])*linear_drag_coefficient[1]*np.sin(euler[0])*thrust*v[2]
        return dragacc1, dragacc2

    def dyn_function(self, states, controls, ext_f):
        euler = states[6:]
        roll_ref = controls[0]
        pitch_ref = controls[1]
        thrust = controls[2]
        dragacc1, dragacc2 = self.aero_drag(states, controls)
        # dynamic of the system
        rhs = states[3:6]
        rhs = ca.horzcat(rhs, (np.cos(euler[0])*np.cos(euler[2])*np.sin(euler[1]) + np.sin(euler[0])*np.sin(euler[2]))*thrust - dragacc1 + ext_f[0])
        rhs = ca.horzcat(rhs, (np.cos(euler[0])*np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.sin(euler[0]))*thrust - dragacc2 + ext_f[1])
        rhs = ca.horzcat(rhs,  (-self.g_ + np.cos(euler[1])*np.cos(euler[0])*thrust + ext_f[2]))
        rhs = ca.horzcat(rhs, (self.roll_gain*roll_ref- euler[0])/self.roll_tau)
        rhs = ca.horzcat(rhs, (self.pitch_gain*pitch_ref - euler[1])/self.pitch_tau)
        rhs = ca.horzcat(rhs,  0.0)
        return rhs

    def dyn_np_function(self, states, controls, ext_F):
        euler = states[6:]
        roll_ref = controls[0]
        pitch_ref = controls[1]
        thrust = controls[2]
        dragacc1, dragacc2 = self.aero_drag(states, controls)
        # dynamic of the system
        rhs = states[3:6]
        rhs = np.concatenate((rhs, (np.cos(euler[0])*np.cos(euler[2])*np.sin(euler[1]) + np.sin(euler[0])*np.sin(euler[2]))*thrust -dragacc1 + ext_F[0]))
        rhs = np.concatenate((rhs, (np.cos(euler[0])*np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.sin(euler[0]))*thrust - dragacc2 + ext_F[1]))
        rhs = np.concatenate((rhs,  -self.g_ + np.cos(euler[1])*np.cos(euler[0])*thrust + ext_F[2]))
        rhs = np.concatenate((rhs, np.array([(self.roll_gain*roll_ref- euler[0])/self.roll_tau])))
        rhs = np.concatenate((rhs, np.array([(self.pitch_gain*pitch_ref - euler[1])/self.pitch_tau])))
        rhs = np.concatenate((rhs,  np.array([0.0])))
        # rhs is in (1, X) shape
        return rhs

    def RK_4(self, s_t_, c_, f_):
        # discretize Runge Kutta 4
         ## approach 1
        # k1 = self.f(s_t_, c_, f_)
        # k2 = self.f(s_t_+self.Ts/2.0*k1, c_, f_)
        # k3 = self.f(s_t_+self.Ts/2.0*k2, c_, f_)
        # k4 = self.f(s_t_+self.Ts*k3, c_, f_)
        ## approach 2
        k1 = self.dyn_function(s_t_, c_, f_)
        k2 = self.dyn_function(s_t_+self.Ts/2.0*k1, c_, f_)
        k3 = self.dyn_function(s_t_+self.Ts/2.0*k2, c_, f_)
        k4 = self.dyn_function(s_t_+self.Ts*k3, c_, f_)
        return s_t_ + self.Ts/6.0*(k1 + 2.0*k2 + 2.0*k3 + k4)

    def objective_function(self,):
        # end state cost
        obj = ca.mtimes([
            (self.states[-1, :6] - self.X_ref[-1, :6]),
            self.P_m,
            (self.states[-1, :6] - self.X_ref[-1, :6]).T
            ])
        # control cost
        for i in range(self.horizon):
            temp_ = ca.horzcat(self.controls[i, :2], np.cos(self.states[i, 6])*np.cos(self.states[i, 7])*self.controls[i, 2]- self.g_)
            obj = obj +  ca.mtimes([
                # self.controls[i, :], self.R_m, self.controls[i, :].T
                temp_, self.R_m, temp_.T
                ])
        # state cost
        for i in range(self.horizon):
            obj = obj + ca.mtimes([
                (self.states[i, :-1] - self.X_ref[i+1, :-1]),
                self.Q_m,
                (self.states[i, :-1] - self.X_ref[i+1, :-1]).T
                ])
        return obj

    def model_based_movement(self, state, control, ext_F, t0, u_, x_):
        # print('state at t {0} is {1}'.format(t0, state))
        # print('control at t {0} is {1}'.format(t0, control))
        k1 = self.dyn_np_function(state, control, ext_F)
        k2 = self.dyn_np_function(state+self.Ts/2.0*k1.T, control, ext_F)
        k3 = self.dyn_np_function(state+self.Ts/2.0*k2.T, control, ext_F)
        k4 = self.dyn_np_function(state+self.Ts*k3.T, control, ext_F)
        x_next = state + self.Ts/6.0*(k1.T+2.0*k2.T+2.0*k3.T+k4.T)
        # x_next = state + self.dyn_np_function(state, control, ext_F)*self.Ts
        # print('nt is {0}'.format(x_next))
        next_cmd_ = np.concatenate((u_[1:], u_[-1:]), axis=0)
        next_s_ = np.concatenate((x_[1:], x_[-1:]), axis=0)
        # print('next_cmd is {0}'.format(next_cmd_))
        # print('next_s is {0}'.format(next_s_))
        return t0+self.Ts, x_next, next_cmd_, next_s_

    def vertical_trajectory(self, current_state,):
        if current_state[2] >= self.trajectory[0, 2]:
            self.trajectory = np.concatenate((current_state.reshape(1, -1), self.trajectory[2:], self.trajectory[-1:]))
        return self.trajectory

    def circle_trajectory(self, current_state, iter):
        if iter<=30:
            if current_state[2] >= self.trajectory[0, 2]:
                self.trajectory = np.concatenate((current_state.reshape(1, -1), self.trajectory[2:], self.trajectory[-1:]))
        else:
            idx_ = np.array([(iter+i-30)/360.0*np.pi for i in range(20)])
            trajectory_ =  self.trajectory[1:].copy()
            trajectory_[:, :2] = np.concatenate((np.cos(idx_), np.sin(idx_))).reshape(2, -1).T
            self.trajectory = np.concatenate((current_state.reshape(1, -1), trajectory_))
            # print(iter)
            # print(trajectory_[:4])
        return self.trajectory

if __name__ == '__main__':
    N = 20
    n_states = 9
    n_controls = 3
    dt = 0.33
    mpc_obj = MPC_opt(state_dim=n_states, dt=dt, N=N)
    init_state = np.array([0.0]*n_states)
    current_state = init_state.copy()
    opt_commands = np.zeros((N, n_controls))
    next_states = np.zeros((N+1, n_states))
    init_trajectory = np.array(
                [[0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.4, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.1, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.1, 0.0, 0.67, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.1, 0.0, 0.69, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, 0.0, 0.73, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, 0.0, 0.76, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, 0.0, 0.83, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, 0.0, 0.85, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.3, 0.0, 0.88, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.3, 0.0, 0.91, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.4, 0.0, 0.93, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.95, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.97, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.7, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.8, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.9, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                ])
    mpc_obj.trajectory = init_trajectory.copy()
    next_trajectories = init_trajectory.copy()
    ext_forces = np.zeros((3, 1))
    # for saving data
    t0 = 0
    x_c = []
    u_c = []
    t_c = []
    x_states = []

    traj_c = []
    # start MPC
    sim_time = 40 # s
    mpc_iter = 0
    index_time = []
    start_time = time.time()

    ## set parameters
    mpc_obj.opti.set_value(mpc_obj.X_ref, next_trajectories)
    mpc_obj.opti.set_value(mpc_obj.external_forces, ext_forces)
    ## initial guess of the optimization targets
    mpc_obj.opti.set_initial(mpc_obj.states, next_states)
    mpc_obj.opti.set_initial(mpc_obj.controls, opt_commands)
    sol = mpc_obj.opti.solve()
    lam_g0_ = sol.value(mpc_obj.opti.lam_g)
    print(sol.stats()["iter_count"])

    while(mpc_iter < sim_time/dt and mpc_iter < 100):
        ## set parameters
        mpc_obj.opti.set_value(mpc_obj.X_ref, next_trajectories)
        mpc_obj.opti.set_value(mpc_obj.external_forces, ext_forces)
        ## initial guess of the optimization targets
        # mpc_obj.opti.set_initial(mpc_obj.states, next_states)
        # mpc_obj.opti.set_initial(mpc_obj.controls, opt_commands)
        mpc_obj.opti.set_initial(sol.value_variables())
        # mpc_obj.opti.set_initial(mpc_obj.opti.lam_g, lam_g0_)
        ## solve the problem
        t_ = time.time()
        sol = mpc_obj.opti.solve()
        index_time.append(time.time() - t_)

        # print(sol.stats()["iter_count"])
        ## get results
        mpc_u_ = sol.value(mpc_obj.controls)
        mpc_x_ = sol.value(mpc_obj.states)
        lam_g0_ = sol.value(mpc_obj.opti.lam_g)
        # print(next_states)
        ## save results
        u_c.append(mpc_u_[0, :])
        t_c.append(t0)
        x_c.append(current_state)
        x_states.append(mpc_x_)
        ## shift the movements, in the experiment, obtaining data from the
        ## the localization system
        # print('current 0 {}'.format(current_state))
        # print('control {}'.format(mpc_u_[:3]))
        t0, current_state, opt_commands, next_states = mpc_obj.model_based_movement(current_state, mpc_u_[0, :], ext_forces, t0, mpc_u_, mpc_x_)
        # next_trajectories = mpc_obj.vertical_trajectory(current_state)
        next_trajectories = mpc_obj.circle_trajectory(current_state, mpc_iter)
        traj_c.append(next_trajectories[1])
        # print('current {}'.format(current_state))
        # print('next_t {}'.format(next_trajectories[:3]))
        # print('x_c {}'.format(mpc_x_[:3]))
        mpc_iter += 1
    print((time.time() - start_time)/mpc_iter)
    print(np.array(index_time).mean())
    print('max iter time {}'.format(np.max(index_time)))
    traj_s = np.array(x_c)
    traj_d = np.array(traj_c)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(traj_s[:, 0], traj_s[:, 1], traj_s[:, 2], 'b')
    ax.plot(traj_d[:, 0], traj_d[:, 1], traj_d[:, 2], 'r')
    plt.show()
