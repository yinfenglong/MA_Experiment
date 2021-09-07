#!/usr/env python

import casadi as ca
import casadi.tools as ca_tools
import numpy as np
from scipy import linalg
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MPC_test():
    def __init__(self, state_dim, dt=0.33, N=20):
        self.Ts = dt
        self.horizon = N
        self.linear_drag_coefficient = [0.01, 0.01]
        self.g_ = 9.8066
        # control relevant parameters
        self.roll_tau = 0.257
        self.roll_gain = 0.75
        self.pitch_tau = 0.259
        self.pitch_gain = 0.78
        self.trajectory = None
        # np.array(
        #         [[0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.0, 0.0, 0.4, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
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
        #         [0.1, 0.0, 0.91, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 0.93, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 0.95, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 0.97, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #         [0.10, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #         ])

        # additional parameters
        self.Q_m = np.diag([80.0, 80.0, 120.0, 80.0, 80.0, 100.0, 10.0, 10.0]) # position, velocity, roll, pitch, (not yaw)
        self.P_m = np.diag([ 86.21, 86.21, 120.95, 6.94, 6.94, 11.04]) # only p and v
        self.P_m[0, 3] = 6.45
        self.P_m[3, 0] = 6.45
        self.P_m[1, 4] = 6.45
        self.P_m[4, 1] = 6.45
        self.P_m[2, 5] = 10.95
        self.P_m[5, 2] = 10.95
        # self.P_m = self.P_m
        self.R_m = np.diag([50.0, 60.0, 1.0]) # roll_ref, pitch_ref, thrust

        # model states
        ## control parameters
        controls = ca_tools.struct_symSX(['roll_ref', 'pitch_ref', 'thrust'])
        roll_ref, pitch_ref, thrust = controls[...]
        # num_controls = controls.size
        ## declare model variables
        states = ca_tools.struct_symSX([
                ca_tools.entry('x', shape=3),
                ca_tools.entry('v', shape=3),
                ca_tools.entry('euler', shape=3)
               ])
        x, v, euler = states[...]
        ## external parameters
        p = ca_tools.struct_symSX([
            (
                ca_tools.entry('F_ext', shape=3)
            )
            ])
        external_forces_, = p[...] # note that X_ref_ is List
        # ##  aero dynamic drag acceleration
        dragacc1, dragacc2 = self.aero_drag_str(states, controls)
        rhs = ca_tools.struct_SX(states)
        rhs['x'] = v
        rhs['v'] = ca.vcat([(np.cos(euler[0])*np.cos(euler[2])*np.sin(euler[1]) + np.sin(euler[0])*np.sin(euler[2]))*thrust - dragacc1 + external_forces_[0], (np.cos(euler[0])*np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.sin(euler[0]))*thrust - dragacc2 + external_forces_[1], self.g_ + np.cos(euler[1])*np.cos(euler[0])*thrust + external_forces_[2]])
        rhs['euler'] = ca.vcat([(self.roll_gain*roll_ref- euler[0])/self.roll_tau, (self.pitch_gain*pitch_ref - euler[1])/self.pitch_tau, 0.0])
        self.f = ca.Function('f', [states, controls, p], [rhs])
        # dragacc1 = np.sin(euler[1])*thrust*v[2] + np.cos(euler[1])*np.cos(euler[2])*self.linear_drag_coefficient[0]*thrust*v[0] - np.cos(euler[1])*self.linear_drag_coefficient[1]*np.sin(euler[2])*thrust*v[1]
        # dragacc2 = (np.cos(euler[0])*np.sin(euler[2])-np.cos(euler[2])*np.sin(euler[0])*np.sin(euler[1]))*self.linear_drag_coefficient[1]*thrust*v[0] - (np.cos(euler[0])*np.cos(euler[2]) + np.sin(euler[1])*np.sin(euler[0])*np.sin(euler[2]))*self.linear_drag_coefficient[1]*thrust*v[1] - np.cos(euler[1])*self.linear_drag_coefficient[1]*np.sin(euler[0])*thrust*v[2]

        # ## create ode function based right hand side
        # rhs = ca_tools.struct_SX(states)
        # rhs['x'] = v
        # rhs['v'] = ca.vertcat(*[(np.cos(euler[0])*np.cos(euler[2])*np.sin(euler[1]) + np.sin(euler[0])*np.sin(euler[2]))*thrust - dragacc1 + external_forces_[0], (np.cos(euler[0])*np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.sin(euler[0]))*thrust - dragacc2 + external_forces_[1], self.g_ + np.cos(euler[1])*np.cos(euler[0])*thrust + external_forces_[2]])
        # rhs['euler'] = ca.vertcat(*[(self.roll_gain*roll_ref- euler[0])/self.roll_tau, (self.pitch_gain*pitch_ref - euler[1])/self.pitch_tau, 0.0])
        # ## define ode function states, controls, external_forces_ --> rhs
        # f = ca.Function('f', [states, controls, external_forces_], [rhs])
        # ## discretize Runge Kutta 4
        # k1 = f(states, controls, external_forces_)
        # k2 = f(states+self.Ts/2.0*k1, controls, external_forces_)
        # k3 = f(states+self.Ts/2.0*k2,  controls, external_forces_)
        # k4 = f(states+self.Ts*k3, controls, external_forces_)
        # new_states = states + self.Ts/6.0*(k1 + 2.0*k2 + 2.0*k3 + k4)
        # ## generate discrete function
        # phi = ca.Function('phi', [states, controls, external_forces_], [new_states])

        # MPC
        self.optimizing_target = ca_tools.struct_symSX([
            (
            ca_tools.entry('U', repeat=self.horizon-1, struct=controls),
            ca_tools.entry('X', repeat=self.horizon, struct=states)
            )
        ])
        U, X, = self.optimizing_target[...] # U is in type List

        self.parameters = ca_tools.struct_symSX([
            (
            ca_tools.entry('X_ref', repeat=self.horizon, struct=states),
            ca_tools.entry('ext_forces', shape=3)
            )
        ])
        X_ref, ext_forces, = self.parameters[...]
        ## define the objective function
        ### end term
        obj = ca.mtimes(
            [   (X[-1][:6] - X_ref[-1][:6]).T,
                self.P_m,
                (X[-1][:6] - X_ref[-1][:6])
            ]
        )
        ### control cost
        for i in range(self.horizon-1):
            temp_ = ca.vertcat(U[i][:2], np.cos(X[i][6])*np.cos(X[i][7])*U[i][2]- self.g_)
            obj += ca.mtimes([
                temp_.T, self.R_m, temp_
             ])
        ### state cost
        for i in range(self.horizon-1):
            temp_ = X[i][:-1] - X_ref[i+1][:-1]
            obj += ca.mtimes([
                temp_.T, self.Q_m, temp_
            ])
        ## constraints (note that this constraints can be updated during the iteration
        g = []
        g.append(X[0] - X_ref[0])
        for i in range(N-1):
            # x_next_ = self.RK_4(X[i], U[i], ext_forces)
            x_next_ = self.dyn_function(X[i], U[i], ext_forces) + X[i]
            g.append(X[i+1] - x_next_)

        # # define nlp problem
        nlp_def = {'x':self.optimizing_target, 'f':obj, 'p':self.parameters, 'g':ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter':200, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
        self.solver = ca.nlpsol('nlp_solver', 'ipopt', nlp_def, opts_setting)

        ## test solver problem
        # lbx = self.optimizing_target(-ca.inf)
        # ubx = self.optimizing_target(ca.inf)
        # lbx['U', :, 'roll_ref'] = -np.deg2rad(45)
        # lbx['U', :, 'pitch_ref'] = -np.deg2rad(45)
        # lbx['U', :, 'thrust'] = 0.5*self.g_
        # ubx['U', :, 'roll_ref'] = np.deg2rad(45)
        # ubx['U', :, 'pitch_ref'] = np.deg2rad(45)
        # ubx['U', :, 'thrust'] = 1.5*self.g_
        # lbg = 0.0
        # ubg = 0.0
        # # lbx = []
        # # ubx = []
        # # for _ in range(self.horizon-1):
        # #     lbx = lbx + [-np.deg2rad(45), -np.deg2rad(45), self.g_*0.5] + [-np.inf]*state_dim
        # #     ubx = ubx + [np.deg2rad(45), np.deg2rad(45), self.g_*1.5] + [np.inf]*state_dim
        # # lbx = lbx + [-np.inf]*state_dim
        # # ubx = ubx + [np.inf]*state_dim
        # # for _ in range(self.horizon-1):
        # #     lbx = lbx + [np.radians(-45), np.radians(-45), 0.5*self.g_]
        # #     ubx = ubx + [np.radians(45), np.radians(45), 1.5*self.g_]
        # # for _ in range(self.horizon):
        # #     lbx = lbx + [-np.inf]*state_dim
        # #     ubx = ubx + [np.inf]*state_dim
        # c_p = self.parameters(0)
        # init_state = self.optimizing_target(0)
        # print(init_state)
        # c_p['X_ref', lambda x:ca.horzcat(*x)] = self.trajectory.T
        # c_p['ext_forces'] = np.array([0.0, 0.0, 0.0])
        # u0 = np.zeros((self.horizon-1, 3))
        # x0 = np.zeros((self.horizon, state_dim))
        # init_state['X', lambda x: ca.horzcat(*x)] = x0.T
        # init_state['U', lambda x: ca.horzcat(*x)] = u0.T

        # solver = self.nlp_solver(x0=init_state, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=c_p)#
        # mpc_estimated = solver['x'].full()
        # u_res, x_res = self.structure_results(mpc_estimated, n_c=num_controls, n_s=state_dim)
        # print(u_res.T)
        # print(x_res.T)

    def structure_results(self, data, n_c, n_s):
        temp_1 = data[:-n_s].reshape(-1, n_c+n_s)
        u_ = temp_1[:, :n_c].T
        s_ = temp_1[:, n_c:].T
        s_ = np.concatenate((s_, data[-n_s:].reshape(n_s, 1)), axis=1)
        return u_, s_

    def aero_drag(self, states, controls):
        # aero dynamic drag acceleration
        linear_drag_coefficient = [0.01, 0.01]
        thrust = controls[2]
        v = states[3:6]
        euler = states[6:]
        dragacc1 = np.sin(euler[1])*thrust*v[2] + np.cos(euler[1])*np.cos(euler[2])*linear_drag_coefficient[0]*thrust*v[0] - np.cos(euler[1])*linear_drag_coefficient[1]*np.sin(euler[2])*thrust*v[1]
        dragacc2 = (np.cos(euler[0])*np.sin(euler[2])-np.cos(euler[2])*np.sin(euler[0])*np.sin(euler[1]))*linear_drag_coefficient[1]*thrust*v[0] - (np.cos(euler[0])*np.cos(euler[2]) + np.sin(euler[1])*np.sin(euler[0])*np.sin(euler[2]))*linear_drag_coefficient[1]*thrust*v[1] - np.cos(euler[1])*linear_drag_coefficient[1]*np.sin(euler[0])*thrust*v[2]
        return dragacc1, dragacc2
    
    def aero_drag_str(self, states, controls):
        # aero dynamic drag acceleration, with the format of structure
        linear_drag_coefficient = [0.01, 0.01]
        thrust = controls['thrust']
        v = states['v']
        euler = states['euler']
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
        return ca.vcat(rhs)

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
        k1 = self.f(s_t_, c_, f_)
        k2 = self.f(s_t_+self.Ts/2.0*k1, c_, f_)
        k3 = self.f(s_t_+self.Ts/2.0*k2, c_, f_)
        k4 = self.f(s_t_+self.Ts*k3, c_, f_)
        ## approach 2
        # k1 = self.dyn_function(s_t_, c_, f_)
        # k2 = self.dyn_function(s_t_+self.Ts/2.0*k1, c_, f_)
        # k3 = self.dyn_function(s_t_+self.Ts/2.0*k2, c_, f_)
        # k4 = self.dyn_function(s_t_+self.Ts*k3, c_, f_)

        result_ = s_t_ + self.Ts/6.0*(k1 + 2.0*k2 + 2.0*k3 + k4)
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



if __name__ == '__main__':
    # model parameters
    n_states = 9 # [x v e ]
    obs_state_size = 7 # [x, q]

    N = 20
    n_controls = 3
    dt = 0.33

    # create an MPC object
    mpc_obj = MPC_test(state_dim=n_states, dt=0.33, N=N)
    init_state = np.array([0.0]*n_states)
    current_state = init_state.copy()
    opt_commands = np.zeros((N-1, n_controls))
    next_states = np.zeros((N, n_states))
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
                [0.9, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                ])
    mpc_obj.trajectory = init_trajectory.copy()
    next_trajectories = init_trajectory.copy()
    ext_forces = np.zeros((3, 1))
    lbg = 0.0
    ubg = 0.0
    lbx = mpc_obj.optimizing_target(-ca.inf)
    ubx = mpc_obj.optimizing_target(ca.inf)
    lbx['U', :, 'roll_ref'] = -np.deg2rad(45)
    lbx['U', :, 'pitch_ref'] = -np.deg2rad(45)
    lbx['U', :, 'thrust'] = 0.5*9.8066
    ubx['U', :, 'roll_ref'] = np.deg2rad(45)
    ubx['U', :, 'pitch_ref'] = np.deg2rad(45)
    ubx['U', :, 'thrust'] = 1.5*9.8066
    # for saving data
    t0 = 0
    x_c = []
    u_c = []
    t_c = []
    x_states = []
    traj_c = []

    # start MPC
    sim_time = 30 # s
    mpc_iter = 0
    index_time = []
    start_time = time.time()
    c_p = mpc_obj.parameters(0)
    init_control = mpc_obj.optimizing_target(0)
    while(mpc_iter < sim_time/dt and mpc_iter < 100):
        ## set parameters
        c_p['X_ref', lambda x:ca.horzcat(*x)] = mpc_obj.trajectory.T
        c_p['ext_forces'] = np.array([0.0, 0.0, 0.0])
        ## initial guess of the optimization targets
        init_control['X', lambda x: ca.horzcat(*x)] = next_states.T
        init_control['U', lambda x: ca.horzcat(*x)] = opt_commands.T
        ## solve the problem
        t_ = time.time()
        sol = mpc_obj.solver(x0=init_control, p=c_p, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)
        index_time.append(time.time() - t_)
        ## get results
        estimated_opt = sol['x'].full()
        mpc_u_ = estimated_opt[:int(n_controls*(N-1))].reshape(N-1, n_controls)
        mpc_x_ = estimated_opt[int(n_controls*(N-1)):].reshape(N, n_states)
        # print(next_states)
        ## save results
        u_c.append(mpc_u_[0, :])
        t_c.append(t0)
        x_c.append(current_state)
        x_states.append(mpc_x_)
        ## shift the movements, in the experiment, obtaining data from the
        ## the localization system
        t0, current_state, opt_commands, next_states = mpc_obj.model_based_movement(current_state, mpc_u_[0, :], ext_forces, t0, mpc_u_, mpc_x_)
        # next_trajectories = mpc_obj.vertical_trajectory(current_state)
        next_trajectories = mpc_obj.circle_trajectory(current_state, mpc_iter)
        traj_c.append(next_trajectories[1])
        # print(next_trajectories[:3])
        # print('current {}'.format(current_state))
        # print('control {}'.format(mpc_u_[0]))
        mpc_iter += 1
    print((time.time() - start_time)/mpc_iter)
    print(np.array(index_time).mean())
    traj_s = np.array(x_c) 
    traj_d = np.array(traj_c)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(traj_s[:, 0], traj_s[:, 1], traj_s[:, 2], 'b')
    ax.plot(traj_d[:, 0], traj_d[:, 1], traj_d[:, 2], 'r')
    plt.show()