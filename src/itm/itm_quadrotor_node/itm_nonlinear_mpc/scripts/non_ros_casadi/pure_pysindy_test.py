#!/usr/bin/
# -*- coding: utf-8 -*-

import numpy as np
import pysindy as ps
import matplotlib.pyplot as plt
import time

# original model
def model(t):
    return None


if __name__ == "__main__":
    t = np.linspace(0, 3, 30)
    x_org = 4 + 3 * np.cos(t) # x_dot = -3* np.sin(t)
    y_org = 2 - 2 * np.sin(t)

    model = ps.SINDy(feature_names=['x', 'y'],
                feature_library=ps.PolynomialLibrary(degree=4),
                optimizer=ps.STLSQ()
                )

    # train the model
    train_x = np.stack((x_org, y_org), axis=1)
    time_ = time.time()
    model.fit(train_x, t=t)
    print('time for training {0}'.format(time.time() - time_))
    model.print()

    # for test
    t_test = np.linspace(3, 4, 10)
    x_gt = 4 + 3 * np.cos(t_test)
    y_gt = 2 - 2 * np.sin(t_test)
    time_ = time.time()
    result = model.simulate(np.array([4 + 3 * np.cos(3), 2 - 2 * np.sin(3)]), t_test)
    print('time for testing {0}'.format(time.time() - time_))
    print(result.shape)
    diff_ = result-np.array([x_gt, y_gt]).T
    print(np.linalg.norm(diff_)/diff_.shape[0])
    print(diff_[:20])
    plt.plot(x_gt, y_gt, 'r--')
    plt.plot(result[:, 0], result[:, 1], 'bo')
    # plt.plot(x_org, y_org, 'g*')
    plt.show()
