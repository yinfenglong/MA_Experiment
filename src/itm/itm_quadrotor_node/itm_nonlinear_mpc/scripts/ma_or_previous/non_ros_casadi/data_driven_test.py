#!/usr/env python
# -*- coding: utf-8 -*-

import pysindy as ps
import numpy as np

# os
from os.path import dirname, join

from draw_and_animation import Animation_2D
class pysindy_estimator(object):
    def __init__(self,):
        pass

if __name__ == "__main__":
    # load training data as dict
    data_file = join(dirname(__file__), './mobile_robot_path.npy')
    train_data = np.load(data_file, allow_pickle=True).item()
    dt = train_data['dt'] # sampling time
    x_states = train_data['x']
    u_states = train_data['u']
    print('we have {0} data for test.'.format(x_states.shape))
    data_length = x_states.shape[0]
    input_length = 100 # save 15 data for each train/prediction
    current_pose = x_states[0] # get the first pose from data

    # create the an object based on pysindy
    # ps_estimator = pysindy_estimator()
    # ps_model = ps.SINDy(feature_names=['x', 'y', 'omega'],
    # feature_library= ps.PolynomialLibrary(degree=2), optimizer=ps.STLSQ())
    ps_c_model = ps.SINDy(feature_names=['x', 'y', 'theta'], feature_library= ps.FourierLibrary(), optimizer=ps.STLSQ())
    ps_model = ps.SINDy(feature_names=['x', 'y', 'theta'], feature_library= ps.FourierLibrary(), optimizer=ps.STLSQ())

    # begin loop
    init_finished = False
    last_trajectory_vec = x_states[0].reshape(1, -1)
    last_control_vec = np.empty(0)
    time_vec= np.zeros(1)
    loop_iter = 1
    prediction_time_vec = np.linspace(0, dt * input_length, input_length)
    trajectory_prediction_poly = []
    trajectory_prediction_pysindy = []
    while data_length - input_length > loop_iter and loop_iter < 101:
        new_state = x_states[loop_iter].reshape(1, -1)
        new_control = u_states[loop_iter-1].reshape(1, -1)
        if init_finished:
            # update training data
            last_trajectory_vec = last_trajectory_vec[1:]
            last_trajectory_vec = np.concatenate((last_trajectory_vec, new_state), axis=0)
            last_control_vec = np.concatenate((last_control_vec, new_control), axis=0)
            time_vec = time_vec[1:]
            time_vec = np.append(time_vec, dt*loop_iter)
            # print('trajectory to fit {}'.format(last_trajectory_vec))
            ## fiting with polynomial method
            ## estimate the parameters
            # p_x = np.polyfit(time_vec, last_trajectory_vec[:, 0], deg=3)
            # p_y = np.polyfit(time_vec, last_trajectory_vec[:, 1], deg=3)
            # p_omega = np.polyfit(time_vec, last_trajectory_vec[:, 2], deg=1)
            ## get the polynomial function
            # x_func = np.poly1d(p_x)
            # y_func = np.poly1d(p_y)
            # omega_func = np.poly1d(p_omega)
            # prediction horizon
            # time_horizon = np.linspace(dt*loop_iter, dt*(loop_iter+input_length), input_length)
            time_horizon = np.array([i*0.02+dt*loop_iter for i in range(15)])
            print("previous time {}".format(time_vec))
            print(time_horizon)
            # poly_pred_x = x_func(time_horizon).reshape(-1, 1)
            # poly_pred_y = y_func(time_horizon).reshape(-1, 1)
            # poly_pred_omega = omega_func(time_horizon).reshape(-1, 1)
            # poly_pred_omega = np.zeros((input_length, 1))
            # temp_ = np.concatenate((poly_pred_x, poly_pred_y, poly_pred_omega), axis=1)
            # print('trajectory after fit {}'.format(temp_))
            ## train pysindy model
            ps_model.fit(last_trajectory_vec, u=last_control_vec, t=dt)
            ## get prediction from pysindy

            ps_result = ps_model.simulate(x0=new_state.reshape(-1), t=time_horizon)
            print("before {0}, current {1} and predicted {2} at index {3}".format(last_trajectory_vec, new_state, ps_result, loop_iter))
            # print(ps_result.shape)
            # trajectory_prediction_poly.append(temp_)
            trajectory_prediction_pysindy.append(ps_result)

        else:
            # not enough data for training, so initialization
            last_trajectory_vec = np.concatenate((last_trajectory_vec, new_state), axis=0)
            time_vec = np.append(time_vec, dt*loop_iter)
            if last_control_vec.shape[0] == 0:
                last_control_vec = new_control.reshape(1, -1)
            else:
                # print(last_control_vec)
                last_control_vec = np.concatenate((last_control_vec, new_control), axis=0)
            if last_trajectory_vec.shape[0] == input_length+1:
                init_finished = True
                # first train for pysindy
                ps_model.fit(last_trajectory_vec, t=time_vec)
                ps_model.print()
                # ps_c_model.fit(last_trajectory_vec[1:], u=last_control_vec, t=dt)
                # ps_c_model.print()
                # print(ps_model)
                # print(new_state.shape)
                temp = ps_model.simulate(last_trajectory_vec[0], time_vec[:30])
                print(temp)
                print(last_trajectory_vec)

        loop_iter += 1
    trajectory_pred_poly = np.array(trajectory_prediction_poly)
    trajectory_pred_pysindy= np.array(trajectory_prediction_pysindy)

    # print(trajectory_pred_poly.shape)
    # draw_result = Animation_2D(robot_states=data_bank[:loop_iter], prediction_trajectory=trajectory_pred_poly)
    # draw_result = Animation_2D(robot_states=x_states[:loop_iter], prediction_trajectory=trajectory_pred_pysindy)