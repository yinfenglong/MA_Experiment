#!/usr/env python

import casadi as ca
import casadi.tools as ca_tools
import numpy as np
import time
from scipy import linalg

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MPC_test():
    def __init__(self, state_dim, dt=0.01, N=20):
        self.Ts = dt
        self.horizon = N
        self.g_ = 9.8066
        # linear_drag_coefficient = [0.01, 0.01]

        # declare model variables
        ## control parameters
        roll_ref_ = ca.SX.sym('roll_ref_')
        pitch_ref_ = ca.SX.sym('pitch_ref_')
        thrust_ref_ = ca.SX.sym('thrust_ref_')
        controls_ = ca.vertcat(*[roll_ref_, pitch_ref_, thrust_ref_])
        num_controls = controls_.size()[0]
        ## control relevant parameters
        self.roll_tau = 0.257
        self.roll_gain = 0.75
        self.pitch_tau = 0.259
        self.pitch_gain = 0.78
        self.trajectory = None
        # self.trajectory = np.array(
        #         [
        #         [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #         ])
        ## model states
        x_ = ca.SX.sym('x_')
        y_ = ca.SX.sym('y_')
        z_ = ca.SX.sym('z_')
        vx_ = ca.SX.sym('vx_')
        vy_ = ca.SX.sym('vy_')
        vz_ = ca.SX.sym('vz_')
        roll_ = ca.SX.sym('roll_')
        pitch_ = ca.SX.sym('pitch_')
        yaw_ = ca.SX.sym('yaw_')
        states_ = ca.vcat([x_, y_, z_, vx_, vy_, vz_, roll_, pitch_, yaw_])
        num_states = states_.size()[0]
        ## external forces
        ext_f_x_ = ca.SX.sym('ext_f_x_')
        ext_f_y_ = ca.SX.sym('ext_f_y_')
        ext_f_z_ = ca.SX.sym('ext_f_z_')
        ext_f_ = ca.vcat([ext_f_x_, ext_f_y_, ext_f_z_])

        ### this is one approach, one can also use directly self.dyn_function to define this work
        dragacc1_, dragacc2_ = self.aero_drag(states_, controls_)
        rhs = [states_[3], states_[4], states_[5]]
        rhs.append((np.cos(roll_)*np.cos(yaw_)*np.sin(pitch_) + np.sin(roll_)*np.sin(yaw_))*thrust_ref_ - dragacc1_ + ext_f_[0])
        rhs.append((np.cos(roll_)*np.sin(pitch_)*np.sin(yaw_) - np.cos(yaw_)*np.sin(roll_))*thrust_ref_ - dragacc2_ + ext_f_[1])
        rhs.append(-self.g_ + np.cos(pitch_)*np.cos(roll_)*thrust_ref_ + ext_f_[2])
        rhs.append((self.roll_gain*roll_ref_- roll_)/self.roll_tau)
        rhs.append((self.pitch_gain*pitch_ref_ - pitch_)/self.pitch_tau)
        rhs.append(0.0)
        self.f = ca.Function('f', [states_, controls_, ext_f_], [ca.vertcat(*rhs)])

        ###### TEST
        # dae = {'x':states_, 'p':ca.vcat([controls_, ext_f_]), 'ode':states_}
        # opts = {'tf':dt}
        # self.f_cvo = ca.integrator('f_cvo', 'cvodes', dae, opts)
        # x_te = ca.SX.sym('x_te', 9)
        # c_te = ca.SX.sym('c_te', 3)
        # f_te = ca.SX.sym('f_te', 3)
        # print(self.f_cvo(x0=x_te, p=ca.vcat([c_te, f_te])))

        ## additional parameters
        # self.external_forces = ca.SX.sym('F_ext', 3)
        self.Q_m = np.diag([80.0, 80.0, 120.0, 80.0, 80.0, 100.0, 10.0, 10.0]) # position, velocity, roll, pitch, (not yaw)
        # self.P_m = self.estimated_penalty_end_term()  # not working yet
        self.P_m = np.diag([86.21, 86.21, 120.95, 6.94, 6.94, 11.04]) # only p and v
        self.P_m[0, 3] = 6.45
        self.P_m[3, 0] = 6.45
        self.P_m[1, 4] = 6.45
        self.P_m[4, 1] = 6.45
        self.P_m[2, 5] = 10.95
        self.P_m[5, 2] = 10.95
        self.R_m = np.diag([50.0, 60.0, 1.0]) # roll_ref, pitch_ref, thrust



        # MPC
        ## states and parameters
        U = ca.SX.sym('U', num_controls, self.horizon-1)
        X = ca.SX.sym('X', num_states, self.horizon)
        X_ref = ca.SX.sym('X_ref', num_states, self.horizon)
        F_ext = ca.SX.sym('F_ext', 3)
        ## constraints and cost
        ### end term
        obj = ca.mtimes([
            (X[:6, -1] - X_ref[:6, -1]).T,
            self.P_m,
            X[:6, -1] - X_ref[:6, -1]
        ])
        ### control cost
        for i in range(self.horizon-1):
            temp_ = ca.vertcat(U[:2, i], np.cos(X[6, i])*np.cos(X[7, i])*U[2, i] - self.g_)
            obj = obj + ca.mtimes([
                temp_.T, self.R_m, temp_
            ])

        ### state cost
        for i in range(self.horizon-1):
            temp_ = X[:-1, i] - X_ref[:-1, i+1]
            obj = obj + ca.mtimes([temp_.T, self.Q_m, temp_])

        ### constraints
        g = []
        g.append(X[:, 0]- X_ref[:, 0])
        for i in range(self.horizon-1):
            x_next_ = self.RK_4(X[:, i], U[:, i], F_ext)
            # x_next_ = self.dyn_function(X[:, i], U[:, i], F_ext) + X[:, i]
            g.append(X[:, i+1]-x_next_)

        opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
        opt_params = ca.vertcat(F_ext, ca.reshape(X_ref, -1, 1))
        nlp_prob = {'f': obj, 'x':opt_variables, 'p':opt_params, 'g':ca.vertcat(*g)}

        opts_setting = {'ipopt.max_iter':200, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6, 'ipopt.warm_start_init_point':'no'}

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        # ##### following only for test
        # ## define constraints
        # lbg = 0.0
        # ubg = 0.0
        # lbx = []
        # ubx = []
        # for _ in range(self.horizon-1):
        #     lbx = lbx + [np.radians(-45), np.radians(-45), 0.5*self.g_]
        #     ubx = ubx + [np.radians(45), np.radians(45), 1.5*self.g_]
        # for _ in range(self.horizon):
        #     lbx = lbx + [-np.inf]*state_dim
        #     ubx = ubx + [np.inf]*state_dim
        # u0 = np.zeros((self.horizon-1, num_controls))
        # x0 = np.zeros((self.horizon, num_states))
        # init_control = ca.vertcat(u0.reshape(-1, 1), x0.reshape(-1, 1))
        # external_forces = np.array([0.0, 0.0, 0.0])
        # c_p = ca.vertcat(external_forces.reshape(-1, 1), self.trajectory.reshape(-1, 1))
        # res = self.solver(x0=init_control, p=c_p, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)
        # estimated_opt = res['x'].full()
        # u1 = estimated_opt[:int(num_controls*(self.horizon-1))].reshape(self.horizon-1, num_controls)
        # x1 = estimated_opt[int(num_controls*(self.horizon-1)):].reshape(self.horizon, num_states)
        # print(u1)
        # print(x1)

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
        rhs = [states[3], states[4], states[5]]
        rhs.append((np.cos(euler[0])*np.cos(euler[2])*np.sin(euler[1]) + np.sin(euler[0])*np.sin(euler[2]))*thrust - dragacc1 + ext_f[0])
        rhs.append((np.cos(euler[0])*np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.sin(euler[0]))*thrust - dragacc2 + ext_f[1])
        rhs.append(-self.g_ + np.cos(euler[1])*np.cos(euler[0])*thrust + ext_f[2])
        rhs.append((self.roll_gain*roll_ref- euler[0])/self.roll_tau)
        rhs.append((self.pitch_gain*pitch_ref - euler[1])/self.pitch_tau)
        rhs.append(0.0)
        # rhs is in (X, 1) shape
        return ca.vertcat(*rhs)

    def dyn_np_function(self, states, controls, ext_f):
        euler = states[6:]
        roll_ref = controls[0]
        pitch_ref = controls[1]
        thrust = controls[2]
        # dynamic of the system
        rhs = states[3:6]
        rhs = np.concatenate((rhs, (np.cos(euler[0])*np.cos(euler[2])*np.sin(euler[1]) + np.sin(euler[0])*np.sin(euler[2]))*thrust+ ext_f[0]))
        rhs = np.concatenate((rhs, (np.cos(euler[0])*np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.sin(euler[0]))*thrust + ext_f[1]))
        rhs = np.concatenate((rhs,  -self.g_ + np.cos(euler[1])*np.cos(euler[0])*thrust + ext_f[2]))
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

        result_ = s_t_ + self.Ts/6.0*(k1 + 2.0*k2 + 2.0*k3 + k4)
        ## approach 3 using cov, working since integrate could not handle sx
        # result_ = self.f_cvo(x0=s_t_, p=ca.vcat([c_, f_]))['xf']

        return result_

    def model_based_movement(self, state, control, ext_F, t0, u_, x_):
        # print('state at t {0} is {1}'.format(t0, state))
        # print('control at t {0} is {1}'.format(t0, control))
        k1 = self.dyn_np_function(state, control, ext_F)
        k2 = self.dyn_np_function(state+self.Ts/2.0*k1.T, control, ext_F)
        k3 = self.dyn_np_function(state+self.Ts/2.0*k2.T, control, ext_F)
        k4 = self.dyn_np_function(state+self.Ts*k3.T, control, ext_F)
        x_next = state + self.Ts/6.0*(k1.T+2.0*k2.T+2.0*k3.T+k4.T)
        # nt_ = state + self.dyn_np_function(state, control, ext_F)*self.Ts
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

    # def circle_trajectory(self, current_state, iter):
    #     if iter<=30:
    #         if current_state[2] >= self.trajectory[0, 2]:
    #             self.trajectory = np.concatenate((current_state.reshape(1, -1), self.trajectory[2:], self.trajectory[-1:]))
    #     else:
    #         idx_ = np.array([(iter+i-30)/360.0*np.pi for i in range(19)])
    #         trajectory_ =  self.trajectory[1:].copy()
    #         trajectory_[:, :2] = np.concatenate((np.cos(idx_), np.sin(idx_))).reshape(2, -1).T
    #         self.trajectory = np.concatenate((current_state.reshape(1, -1), trajectory_))
    #         # print(iter)
    #         # print(trajectory_[:4])
    #     return self.trajectory
    def circle_trajectory(self, current_state, iter):
        if iter<=30:
            if current_state[2] >= self.trajectory[0, 2]:
                self.trajectory = np.concatenate((current_state.reshape(1, -1), self.trajectory[2:], self.trajectory[-1:]))
        else:
            idx_ = np.array([(iter+i-30)/360.0*np.pi for i in range(19)])
            trajectory_ =  self.trajectory[1:].copy()
            trajectory_[:, :2] = np.concatenate((np.cos(idx_), np.sin(idx_))).reshape(2, -1).T
            self.trajectory = np.concatenate((current_state.reshape(1, -1), trajectory_))
            # print(iter)
            # print(trajectory_[:4])
        return self.trajectory
    # def estimated_penalty_end_term(self):
    #     A = np.diag([1.0, 1.0, 1.0,
    #                 -self.linear_drag_coefficient[0], -self.linear_drag_coefficient[1], 0.0])
    #     B = np.zeros((6, 3))
    #     B[3, 1] = 9.8066
    #     B[4, 0] = -9.8066
    #     B[5, 2] = 1.0
    #     Q = self.Q_m[:6, :6]
    #     R = self.Q_m[8:, 8:]
    #     print("{0}, {1}, {2}, {3}".format(A, B, Q, R))
    #     return linalg.solve_continuous_are(A, B, Q, R)


if __name__ == '__main__':
    # model parameters
    n_states = 9 # [x v e ]
    N = 20
    n_controls = 3
    dt = 0.01
    # create an MPC object
    mpc_obj = MPC_test(state_dim=n_states, dt=0.33, N=N)
    init_state = np.array([0.0]*n_states)
    current_state = init_state.copy()
    opt_commands = np.zeros((N-1, n_controls))
    next_states = np.zeros((N, n_states))
    # init_trajectory = np.array(
    #             [[0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #              [0.0, 0.0, 0.4, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
    #             [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.1, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.1, 0.0, 0.67, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.1, 0.0, 0.69, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.2, 0.0, 0.73, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.2, 0.0, 0.76, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.2, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.2, 0.0, 0.83, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.2, 0.0, 0.85, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.3, 0.0, 0.88, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.3, .2, 0.91, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.4, 0.2, 0.93, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.5, 0.2, 0.95, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.5, 0.2, 0.97, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.7, 0.2, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.8, 0.2, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.9, 0.2, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #             [0.9, 0.2, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #             ])
    init_trajectory = np.array(
                [[0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
    ext_forces = np.array([0.0, 0.0, -0.1]).reshape(-1, 1)
    lbg = 0.0
    ubg = 0.0
    lbx = []
    ubx = []
    for _ in range(N-1):
        lbx = lbx + [np.deg2rad(-45), np.deg2rad(-45), 0.5*9.8066]
        ubx = ubx + [np.deg2rad(45), np.deg2rad(45), 1.5*9.8066]
    for _ in range(N):
        lbx = lbx + [-np.inf]*n_states
        ubx = ubx + [np.inf]*n_states
    # for saving data
    t0 = 0
    x_c = []
    u_c = []
    t_c = []
    x_states = []
    traj_c = []

    # start MPC
    sim_time = 600 # s
    mpc_iter = 0
    index_time = []
    start_time = time.time()
    while(mpc_iter < sim_time/dt and mpc_iter < 10):
        ## set parameters
        control_params = ca.vertcat(ext_forces.reshape(-1, 1), mpc_obj.trajectory.reshape(-1, 1))
        ## initial guess of the optimization targets
        init_control = ca.vertcat(opt_commands.reshape(-1, 1), next_states.reshape(-1, 1))
        ## solve the problem
        t_ = time.time()
        sol = mpc_obj.solver(x0=init_control, p=control_params, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)
        index_time.append(time.time() - t_)
        ## get results
        estimated_opt = sol['x'].full()
        mpc_u_ = estimated_opt[:int(n_controls*(N-1))].reshape(N-1, n_controls)
        mpc_x_ = estimated_opt[int(n_controls*(N-1)):].reshape(N, n_states)
        print(mpc_u_)
        ## save results
        u_c.append(mpc_u_[0, :])
        t_c.append(t0)
        x_c.append(current_state)
        x_states.append(mpc_x_)
        ## shift the movements, in the experiment, obtaining data from the
        ## the localization system
        t0, current_state, opt_commands, next_states = mpc_obj.model_based_movement(current_state, mpc_u_[0, :], ext_forces, t0, mpc_u_, mpc_x_)
        # next_trajectories = mpc_obj.vertical_trajectory(current_state)
        # print(next_states)
        next_trajectories = mpc_obj.circle_trajectory(current_state, mpc_iter)
        traj_c.append(next_trajectories[1])
        # print(next_trajectories[:3])
        # print('current {}'.format(current_state))
        # print('control {}'.format(mpc_u_[0]))
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
