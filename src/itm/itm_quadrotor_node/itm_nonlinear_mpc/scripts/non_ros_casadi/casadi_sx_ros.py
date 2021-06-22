#!/usr/bin/env python

import casadi as ca
import casadi.tools as ca_tools
import numpy as np
import time
from os import system
from scipy import linalg

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MPC_UAV(object):
    def __init__(self, dt=0.3, N=20, init_state_=None, init_control_=None, is_load_lib_=False, lib_path_='./nmpc.so', is_gen_c_code_=False, test_only=False):
        self.Ts = dt
        self.horizon = N
        self.g_ = 9.8066
        # declare model variables
        ## control parameters
        self.num_controls = 3
        ## control relevant parameters
        self.roll_tau = 0.257
        self.roll_gain = 0.75
        self.pitch_tau = 0.259
        self.pitch_gain = 0.78
        self.trajectory = None
        self.num_states = 9
        self.lib_path = lib_path_
        self.is_load_lib = is_load_lib_
        self.is_gen_c_code = is_gen_c_code_
        # self.num_ext_params = 3

        # ### this is one approach, one can also use directly self.dyn_function to define this work

        ## additional parameters
        # self.external_forces = ca.SX.sym('F_ext', 3)
        self.Q_m = np.diag([80.0, 80.0, 120.0, 80.0, 80.0, 100.0, 10.0, 10.0]) # position, velocity, roll, pitch, (not yaw)
        # self.P_m = self.estimated_penalty_end_term()  # not working yet
        self.P_m = np.diag([86.21, 86.21, 120.95, 6.94, 6.94, 11.04]) # only p and v
        # self.P_m[0, 3] = 6.45
        # self.P_m[3, 0] = 6.45
        # self.P_m[1, 4] = 6.45
        # self.P_m[4, 1] = 6.45
        # self.P_m[2, 5] = 10.95
        # self.P_m[5, 2] = 10.95
        self.R_m = np.diag([50.0, 60.0, 1.0]) # roll_ref, pitch_ref, thrust

        if init_state_ is not None:
            self.init_state = init_state_
        else:
            self.init_state = np.array([0.0]*self.num_states)

        if init_control_ is not None:
            self.init_control = init_control_
        else:
            self.init_control = np.array([0.0, 0.0, self.g_])

        self.init_dynamic(test_only)
        print('finish')




        ##### following only for test
        ## define constraints
        if test_only:
            lbg = 0.0
            ubg = 0.0
            lbx = []
            ubx = []
            self.trajectory = np.tile(np.array([0.0, 0.3, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), (20, 1))
            self.trajectory = np.concatenate((np.array([[0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]), self.trajectory), axis=0)
            for _ in range(self.horizon):
                lbx = lbx + [np.radians(-45), np.radians(-45), 0.5*self.g_]
                ubx = ubx + [np.radians(45), np.radians(45), 1.5*self.g_]
            for _ in range(self.horizon+1):
                lbx = lbx + [-np.inf]*9
                ubx = ubx + [np.inf]*9
            u0 = np.zeros((self.horizon, 3))
            x0 = np.zeros((self.horizon+1, 9))
            init_control = ca.vertcat(u0.reshape(-1, 1), x0.reshape(-1, 1))
            external_forces = np.array([0.0, 0.0, 0.0])
            c_p = ca.vertcat(external_forces.reshape(-1, 1), self.trajectory.reshape(-1, 1))
            res = self.solver(x0=init_control, p=c_p, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)
            estimated_opt = res['x'].full()
            u1 = estimated_opt[:int(3*(self.horizon))].reshape(self.horizon, 3)
            x1 = estimated_opt[int(3*(self.horizon)):].reshape(self.horizon+1, 9)
            print(u1)
            print(x1)

    def init_dynamic(self, test_only_):
        # control input
        roll_ref_ = ca.SX.sym('roll_ref_')
        pitch_ref_ = ca.SX.sym('pitch_ref_')
        thrust_ref_ = ca.SX.sym('thrust_ref_')
        controls_ = ca.vertcat(*[roll_ref_, pitch_ref_, thrust_ref_])
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
        ## external forces as optimization parameters
        ext_f_x_ = ca.SX.sym('ext_f_x_')
        ext_f_y_ = ca.SX.sym('ext_f_y_')
        ext_f_z_ = ca.SX.sym('ext_f_z_')
        ext_f_ = ca.vcat([ext_f_x_, ext_f_y_, ext_f_z_])

        dragacc1_, dragacc2_ = self.aero_drag(states_, controls_)
        rhs = [states_[3], states_[4], states_[5]]
        rhs.append((np.cos(roll_)*np.cos(yaw_)*np.sin(pitch_) + np.sin(roll_)*np.sin(yaw_))*thrust_ref_ - dragacc1_ + ext_f_[0])
        rhs.append((np.cos(roll_)*np.sin(pitch_)*np.sin(yaw_) - np.cos(yaw_)*np.sin(roll_))*thrust_ref_ - dragacc2_ + ext_f_[1])
        rhs.append(-self.g_ + np.cos(pitch_)*np.cos(roll_)*thrust_ref_ + ext_f_[2])
        rhs.append((self.roll_gain*roll_ref_- roll_)/self.roll_tau)
        rhs.append((self.pitch_gain*pitch_ref_ - pitch_)/self.pitch_tau)
        rhs.append(0.0)
        self.f = ca.Function('f', [states_, controls_, ext_f_], [ca.vcat(rhs)])

        ## Fold
        F = self.sys_dynamics(self.Ts)
        fMap = F.map(self.horizon, "openmp")

        # MPC
        ## states and parameters
        U = ca.SX.sym('U', self.num_controls, self.horizon)
        X = ca.SX.sym('X', self.num_states, self.horizon+1)
        X_ref = ca.SX.sym('X_ref', self.num_states, self.horizon+1)
        F_ext = ca.SX.sym('F_ext', 3)
        ## constraints and cost
        ### end term
        obj = ca.mtimes([
            (X[:6, -1] - X_ref[:6, -1]).T,
            self.P_m,
            X[:6, -1] - X_ref[:6, -1]
        ])
        ### control cost
        for i in range(self.horizon):
            temp_ = ca.vertcat(U[:2, i], ca.cos(X[6, i])*ca.cos(X[7, i])*U[2, i] - self.g_)
            obj = obj + ca.mtimes([
                temp_.T, self.R_m, temp_
            ])

        ### state cost
        for i in range(1, self.horizon+1):
            temp_ = X[:-1, i] - X_ref[:-1, i] # X[:-1, i] - X_ref[:-1, i+1]
            obj = obj + ca.mtimes([temp_.T, self.Q_m, temp_])

        ### constraints
        g = []
        X_next = fMap(X[:, :self.horizon], U, F_ext)
        g.append(X[:, 0]- X_ref[:, 0])
        for i in range(self.horizon):
            g.append(X[:, i+1]-X_next[:, i])

        opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
        print('opt variables shape {}'.format(opt_variables.shape))
        opt_params = ca.vertcat(F_ext, ca.reshape(X_ref, -1, 1))
        nlp_prob = {'f': obj, 'x':opt_variables, 'p':opt_params, 'g':ca.vcat(g)}

        ipopt_options = {
            'verbose': False,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-4,
            "ipopt.max_iter": 100,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.print_level": 0,
            "print_time": False
        }

        if not test_only_:
            if self.is_load_lib:
                print("Loading pre-complied library")
                self.solver = ca.nlpsol('solver', 'ipopt', self.lib_path, ipopt_options)
                # import_c_so = ca.Importer('./mpc_v1.c', 'clang')
                # self.solver = ca.nlpsol('solver', 'ipopt', import_c_so, ipopt_options)
            else:
                self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, ipopt_options)
                if self.is_gen_c_code:
                    # jit (just-in-time compilation)
                    print("Generating shared library........")
                    cname = self.solver.generate_dependencies("mpc_v1.c")
                    print("c file is generated")
                    print("using 'clang -fPIC -shaped -O3 mpc_v1.c -o nmpc.so' to gerenate JIT library")
                    # system('clang -fPIC -shared -O3 ' + cname + ' -o ' + self.lib_path) # -O3
        else:
            self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, ipopt_options)




    def sys_dynamics(self, dt):
        M = 4
        x_d = ca.SX.sym("xd", self.num_states)
        u_d = ca.SX.sym("ud", self.num_controls)
        p_d = ca.SX.sym("pd", 3)

        X = x_d
        for _ in range(M):
            k1 = dt*self.f(X, u_d, p_d)
            k2 = dt*self.f(X+0.5*k1, u_d, p_d)
            k3 = dt*self.f(X+0.5*k2, u_d, p_d)
            k4 = dt*self.f(X+k3, u_d, p_d)
            X = X + (k1+2*k2+2*k3+k4)/6
        F = ca.Function('F', [x_d, u_d, p_d], [X])
        return F

    def aero_drag(self, states, controls):
        # aero dynamic drag acceleration
        linear_drag_coefficient = [0.01, 0.01]
        thrust = controls[2]
        v = states[3:6]
        euler = states[6:]
        dragacc1 = np.sin(euler[1])*thrust*v[2] + np.cos(euler[1])*np.cos(euler[2])*linear_drag_coefficient[0]*thrust*v[0] - np.cos(euler[1])*linear_drag_coefficient[1]*np.sin(euler[2])*thrust*v[1]
        dragacc2 = (np.cos(euler[0])*np.sin(euler[2])-np.cos(euler[2])*np.sin(euler[0])*np.sin(euler[1]))*linear_drag_coefficient[1]*thrust*v[0] - (np.cos(euler[0])*np.cos(euler[2]) + np.sin(euler[1])*np.sin(euler[0])*np.sin(euler[2]))*linear_drag_coefficient[1]*thrust*v[1] - np.cos(euler[1])*linear_drag_coefficient[1]*np.sin(euler[0])*thrust*v[2]
        return dragacc1, dragacc2




if __name__ == '__main__':
    print('main')
    # model parameters
    n_states = 9 # [x v e ]
    N = 20
    n_controls = 3
    dt = 0.3
    # create an MPC object
    mpc_obj = MPC_UAV(dt=dt, N=N, is_load_lib_=False, test_only=True, is_gen_c_code_=False)

