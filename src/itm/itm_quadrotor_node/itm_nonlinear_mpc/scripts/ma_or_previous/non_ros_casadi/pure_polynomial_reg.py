#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures

if __name__ == "__main__":
    t = np.linspace(0, 3, 30)
    x_org = 4 + 3 * np.cos(t) # x_dot = -3* np.sin(t)
    y_org = 2 - 2 * np.sin(t)
    diff_x_ = x_org[1:] - x_org[:-1]
    diff_y_ = y_org[1:] - y_org[:-1]
    p_x = np.polyfit(t, x_org, 5)
    p_y = np.polyfit(t, y_org, 5)
    p_vx = np.polyfit(t[1:], diff_x_, 4)
    p_vy = np.polyfit(t[1:], diff_y_, 4)
    # print fitting parameter 
    print("x parameters {0} \n y parameters {1} ".format(p_x, p_y))
    x_f = np.poly1d(p_x,)
    y_f = np.poly1d(p_y,)
    vx_f = np.poly1d(p_vx,)
    vy_f = np.poly1d(p_vy,)

    # print the further results 
    t_test = np.linspace(3, 4, 10)
    x_gt = 4 + 3 * np.cos(t_test)
    y_gt = 2 - 2 * np.sin(t_test)
    x_est = x_f(t_test)
    y_est = y_f(t_test)
    vx_est = vx_f(t_test[1:])
    vy_est = vy_f(t_test[1:])
    print(vx_est)
    dt_ = 0.01 # seconds
    # x_v_est = np.concatenate(([x_est[0]], 0.5 * (x_est[1:]+x_est[:-1] + dt_*vx_est)))
    # y_v_est = np.concatenate(([y_est[0]], 0.5 * (y_est[1:] + y_est[:-1] + dt_*vy_est)))
    x_v_est = np.concatenate(([x_est[0]], x_est[:-1] + dt_*vx_est))
    y_v_est = np.concatenate(([y_est[0]], y_est[:-1] + dt_*vy_est))

    plt.plot(x_gt, y_gt, '-r', x_est, y_est, 'ob')
    plt.plot(x_v_est, y_v_est, '*g',)
    plt.show()