#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-08-12 15:46:05
LastEditors: Wei Luo
LastEditTime: 2021-08-17 15:44:14
Note: Note
'''
import argparse
import numpy as np
import os
import sys
import time

file_path = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(file_path, '../')
sys.path.append(parent_dir)
from traj_gen import poly_trajectory_qp as pt
from traj_gen import uav_poly_trajectory as uav_pt
from traj_gen.traj_utils import handle_dmoc_results

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--npy", type=str,
                        help="load npy file.")
    parser.add_argument("--with_manipulator", type=bool,
                        default=False, help="generate tracking trajectory ")
    parser.add_argument("--output_hz", type=int, default=100.0)

    input_arguments = parser.parse_args()
    saved_data = handle_dmoc_results()

    saved_data.load_saved_data(input_arguments.npy)

    if input_arguments.with_manipulator:
        # estimation for x, y, z, roll, pitch, yaw and alpha (manipulator)
        dim = 7
        ts = np.linspace(start=0.0, stop=saved_data.Tn,
                         num=saved_data.N + 1, endpoint=True).flatten()
        order = [8, 3]
        optimTarget = 'end-derivative'
        maxConti = [4, 1]
        pos_objWeights = np.array([0, 0, 0, 1])
        ang_objWeights = np.array([0, 1])
        pTraj = uav_pt.UAVTrajGen(ts, order, dim_=7, maxContiOrder_=maxConti)
        Xs = saved_data.opt_x
        for i in range(saved_data.opt_x.shape[0]):
            pin_ = {'t': ts[i], 'd': 0, 'X': Xs[i, :7]}
            pTraj.addPin(pin_)
        pTraj.setDerivativeObj(pos_objWeights, ang_objWeights)
        print("solving")
        time_start = time.time()
        pTraj.solve()
        time_end = time.time()
        print(time_end - time_start)
        # we only require the state and its first derivate
        traj_r = pTraj.showTraj(plotOrder=1, N_plot=int(
            saved_data.Tn * input_arguments.output_hz + 1.0), showPlot=True)
        if traj_r is not None:
            np.save("./saved_trajs/uav_DMOC_manipulator_tracking.npy", traj_r)
    else:
        # UAV without manipulator version
        dim = 6  # estimation for x, y, z, roll, pitch and yaw
        ts = np.linspace(start=0.0, stop=saved_data.Tn,
                         num=saved_data.N + 1, endpoint=True).flatten()

        order = [8, 3]
        maxConti = [4, 1]
        pos_objWeights = np.array([0, 0, 0, 1])
        ang_objWeights = np.array([0, 1])
        pTraj = uav_pt.UAVTrajGen(ts, order, dim_=6, maxContiOrder_=maxConti)
        Xs = saved_data.opt_x
        for i in range(saved_data.opt_x.shape[0]):
            pin_ = {'t': ts[i], 'd': 0, 'X': Xs[i, :6]}
            pTraj.addPin(pin_)
        pTraj.setDerivativeObj(pos_objWeights, ang_objWeights)
        print("solving")
        time_start = time.time()
        pTraj.solve()
        time_end = time.time()
        print(time_end - time_start)
        # we only require the state and its first derivate
        traj_r = pTraj.showTraj(plotOrder=1, N_plot=int(
            saved_data.Tn * input_arguments.output_hz + 1.0), showPlot=False)

        ts_u = ts[:-1]
        optimTarget = 'end-derivative'
        uTraj = pt.PolyTrajGen(
            ts_u, 3, algo_='end-derivative', dim_=1, maxContiOrder_=1)
        u = saved_data.opt_u
        for i in range(saved_data.opt_u.shape[0]):
            pin_ = {'t': ts_u[i], 'd': 0, 'X': u[i, :1].flatten()}
            uTraj.addPin(pin_)
        uTraj.setDerivativeObj(np.array([0, 1]))
        print("solving")
        time_start = time.time()
        uTraj.solve()
        time_end = time.time()
        print(time_end - time_start)
        u_r = uTraj.showTraj(plotOrder=1, N_plot=int(
            saved_data.Tn * input_arguments.output_hz + 1.0), showPlot=False)

        if traj_r is not None:
            if u_r is not None:
                traj_r = np.concatenate((traj_r, u_r), axis=1)
            np.save("./saved_trajs/uav_DMOC_simplest_tracking.npy", traj_r)

    # normal xyz version
    # dim = 3 # currently estimation for x, y, z
    # ts = np.linspace(start=0.0, stop=saved_data.Tn,
    #                  num=saved_data.N + 1, endpoint=True).flatten()
    # Xs = saved_data.opt_x
    # print(ts.shape)
    # print(saved_data.opt_x.shape)

    # order = 8
    # optimTarget = 'end-derivative'  # 'end-derivative' 'poly-coeff'
    # maxConti = 4
    # objWeights = np.array([0.0, 0.0, 1])
    # pTraj = pt.PolyTrajGen(ts, order, optimTarget, dim, maxConti)

    # for i in range(saved_data.opt_x.shape[0]):
    #     pin_ = {'t': ts[i], 'd': 0, 'X': Xs[i, :3]}
    #     pTraj.addPin(pin_)

    # # for i in range(saved_data.opt_x.shape[0]):
    # #     pin_ = {'t': ts[i], 'd': 1, 'X': Xs[i, 6:9]}
    # #     pTraj.addPin(pin_)

    # pTraj.setDerivativeObj(objWeights)
    # print("solving")
    # time_start = time.time()
    # pTraj.solve()
    # time_end = time.time()
    # print(time_end - time_start)

    # print("trajectory")
    # pTraj.showTraj(4, N_plot=int(saved_data.Tn * 100.0 + 1.0))
    # print('path')
    # fig_title = 'poly order : {0} / max continuity: {1} / minimized derivatives order: {2}'.format(
    #     pTraj.N, pTraj.maxContiOrder, np.where(pTraj.weight_mask > 0)[0].tolist())
    # traj_3d = pTraj.showPath(
    #     fig_title, N_plot=int(saved_data.Tn * 100.0 + 1.0))

    # print(traj_3d)
