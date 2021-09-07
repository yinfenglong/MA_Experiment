#!/usr/env python
# -*- coding: utf-8 -*-


## this file is to generate a virtual path for leaning

import numpy as np
import sys
import argparse
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def path_generator(idx:int):
    if idx==0:
        pass

def mobile_robot_model(current_pos, control, dt=0.3):
    v = control[0]
    omega = control[1]
    return np.array([current_pos[0]+dt*v*np.cos(current_pos[2]),
        current_pos[1]+dt*v*np.sin(current_pos[2]),
        current_pos[2]+omega*dt])

robot_controls = []
def control_t(t):
    if (t/0.02)%8==0:
        robot_control = np.array([np.random.rand() * 1.0, np.random.rand() * 3.14])
    return robot_control

def mobile_robot_model_new(t, z):
    control = control_t(t)
    robot_controls.append(control)
    v = control[0]
    omega = control[1]
    return np.array([dt*v*np.cos(z[2]),
        dt*v*np.sin(z[2]),
        omega*dt])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Path Generator')
    parser.add_argument('--path_idx', metavar='idx', type=int, default=0, help='path index')
    parser.add_argument('--dT', '-t', type=float, default=0.02, help='update time')
    parser.add_argument('--sim_time', '-st', type=float, default=30, help='simulation time')
    parser.add_argument('--verbose', '-v', type=bool, default=False, help='show the result')
    parser.add_argument('--save_npy', '-s', type=bool, default=False, help='save trajectory as npy file')
    args = parser.parse_args()

    # get user inputs
    sim_time = args.sim_time
    dt = args.dT
    is_verbose = args.verbose

    robot_current_state = np.zeros(3)

    # approach 1 using math model
    control_inputs = []
    robot_states = []
    path = []
    loop_iter = 0
    while(loop_iter < sim_time/dt):
        # begin a loop to generate path
        if loop_iter%8 == 0:
            # every 10 time steps to change a direction
            control = np.array([np.random.rand() * 1.0, np.random.rand() * 3.14])
        control_inputs.append(control)
        robot_states.append(robot_current_state)
        robot_current_state = mobile_robot_model(robot_current_state, control, dt)
        loop_iter += 1
    robot_states.append(robot_current_state)
    if is_verbose:
        robot_traj = np.array(robot_states)
        print(robot_traj)
        plt.plot(robot_traj[:, 0], robot_traj[:, 1], 'ro')
        plt.show()
    data_export = {'x':np.array(robot_states), 'u':np.array(control_inputs), 'loop_length':loop_iter, 'dt':dt}

    # approach 2 using ivp
    # t_train = np.arange(0.0, sim_time, dt)
    # robot_states = odeint(mobile_robot_model_new, robot_current_state, t_train)
    # data_export = {'x':robot_states, 'u':np.array(robot_controls), 'dt':dt}
    # create a dictionary to save data
    # np.save('mobile_robot_path.npy', data_export)



