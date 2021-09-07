#!/usr/bin/env python
# coding=utf-8

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches

class Animation_2D(object):
    def __init__(self, robot_states, prediction_trajectory, export_gif=False, filename='2d_sim'):
        self.robot_states = robot_states
        if self.robot_states.shape[0] > prediction_trajectory.shape[0]:
            # add the initial guess of the trajectory of the
            data_size_diff_ = self.robot_states.shape[0] - prediction_trajectory.shape[0]
            temp_list_ = []
            for i in range(data_size_diff_):
                temp_ = robot_states[i].repeat(prediction_trajectory.shape[1], axis=0).reshape(3, -1).T
                temp_list_.append(temp_)
            self.prediction_trajectory = np.concatenate((temp_list_, prediction_trajectory), axis=0)
        else:
            self.prediction_trajectory = prediction_trajectory

        self.radius = 0.15
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-0.8, 1.2), ylim=(-0.8, 1.2))
        # self.fig.set_dpi(400)
        self.fig.set_size_inches(7, 6.5)
        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)), init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_gif:
            self.ani.save(filename+'.gif', writer='imagemagick', fps=100)
        plt.show()

    def animation_init(self,):
        self.robot_circle = plt.Circle(self.robot_states[0, :2], self.radius, color='b', fill=False)
        self.ax.add_artist(self.robot_circle)
        self.robot_arr = mpatches.Arrow(self.robot_states[0, 0], self.robot_states[0, 1], self.radius * np.cos(self.robot_states[0, 2]), self.radius * np.sin(self.robot_states[0, 2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        self.pred_line, = self.ax.plot(self.prediction_trajectory[0, :, 0], self.prediction_trajectory[0, :, 1], 'ro', )
        return self.robot_circle, self.robot_arr, self.pred_line


    def animation_loop(self, idx_):
        pos = self.robot_states[idx_, :2]
        ori = self.robot_states[idx_, 2]
        self.robot_circle.center = pos
        # self.robot_arr.remove()
        # self.robot_arr = mpatches.Arrow(pos[0], pos[1], self.radius * np.cos(ori), self.radius * np.sin(ori), width=0.2, color='r')
        # self.ax.add_patch(self.robot_arr)
        self.pred_line.set_xdata(self.prediction_trajectory[idx_,:, 0])
        self.pred_line.set_ydata(self.prediction_trajectory[idx_,:, 1])


