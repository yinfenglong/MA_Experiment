#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-05-30 16:21:31
LastEditors: Wei Luo
LastEditTime: 2021-06-07 17:32:23
Note: Note
'''

import numpy as np
from matplotlib import pyplot as plt


if __name__ == '__main__':
    np_file = '/home/arnold/Develop/ROS_ws/ITM_ws/pose_vector_100Hz.npy'
    # np_file = '/home/arnold/DataSSD/NutstoreFiles/Programming/FlightData/20210601/exp_data_y_hand.npy'

    exp_data = np.load(np_file, allow_pickle=True)
    data_length = exp_data.shape[0]

    t = np.arange(0., 0.01*data_length, 0.01)
    x = []
    y = []
    z = []
    roll = []
    pitch = []
    yaw = []
    ref_x = np.array([0.]*t.shape[0])
    for i in range(data_length):
        x.append(exp_data[i][0])
        y.append(exp_data[i][1])
        z.append(exp_data[i][2])
        roll.append(exp_data[i][3])
        pitch.append(exp_data[i][4])
        yaw.append(exp_data[i][5])

    # plt.plot(t, x, 'r--', t, y, 'b--',)
    # plt.plot(t, roll, 'b.-', t,
    #          pitch, 'r.-', t, roll_rate_cmd, 'go-', t, pitch_rate_cmd, 'ko-', markersize=0.9)

    plt.plot(t, x, 'r--', t, pitch, 'b--', t, ref_x, 'k-')
    plt.legend(labels=['x', 'pitch', ], loc='best')
    plt.show()
    plt.plot(t, y, 'r--', t, roll, 'b--', t, ref_x, 'k-')
    plt.legend(labels=['y', 'roll', ], loc='best')
    plt.show()
