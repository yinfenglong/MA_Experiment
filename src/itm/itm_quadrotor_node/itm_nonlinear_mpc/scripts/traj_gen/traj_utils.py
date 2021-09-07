#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-08-05 17:54:31
LastEditors: Wei Luo
LastEditTime: 2021-08-14 16:34:41
Note: Note
'''
import pandas as pd
import numpy as np
import os

from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib import cm
from matplotlib import rc


rc('font', **{'family': 'serif', 'serif': ['Computer Modern Roman']})
# rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
rc('font', **{'size': 11})
rc('text', usetex=True)

# 3D path class


class show3DResults(object):
    def __init__(self, simX, manipulator_states,
                 mobile_robot_states, init_pose, Wp_ref, pickup_point,
                 engage_index_list, l_param, Tn, N, pickup_stick_height=0.1):
        self.X_state = simX
        self.manipulator_states = manipulator_states
        self.mobile_robot_states = mobile_robot_states
        self.init_pose = init_pose
        self.Wp_ref = Wp_ref
        self.pickup_point = pickup_point
        self.engage_index_list = engage_index_list
        self.result_l_param = l_param
        self.num = np.shape(simX)[0]
        self.Tn = Tn
        self.N = int(N)
        self.pickup_stick_height = pickup_stick_height
        # make up the data for ground robot
        self.mobile_robot_states = np.hstack((self.mobile_robot_states[:, :2], self.pickup_point[2] * np.ones(
            (self.mobile_robot_states.shape[0], 1)), self.mobile_robot_states[:, 2:], np.zeros((self.mobile_robot_states.shape[0], 1))))

        self.time_stamps = np.linspace(0.0, self.Tn, self.N + 1, endpoint=True)

    @staticmethod
    def rotation_matrix(angle):
        phi = angle[0]
        theta = angle[1]
        psi = angle[2]
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
                       [np.sin(psi), np.cos(psi), 0],
                       [0, 0, 1]])
        Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
                       [0, 1, 0],
                       [-np.sin(theta), 0, np.cos(theta)]])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(phi), -np.sin(phi)],
                       [0, np.sin(phi), np.cos(phi)]])
        eRb = Rz @ Ry @ Rx

        return eRb

    def cal_heading(self, mode=0):
        # calculate the heading
        # mode:
        # 0 heading
        # 1 speed direction
        # 2 yaw direction
        heading_list = []
        for i in range(self.X_state.shape[0]):
            if mode == 0:
                eRb_0 = self.rotation_matrix(self.X_state[i, 3:6])
                unit_xe = np.array([1, 0, 0]).reshape(-1, 1)
                unit_vec_heading_uav = eRb_0 @ unit_xe
                heading_uav = np.arctan2(
                    unit_vec_heading_uav[1], unit_vec_heading_uav[0])
                heading_uav = np.rad2deg(heading_uav)
            elif mode == 1:
                heading_uav = np.arctan2(
                    self.X_state[i, 7], self.X_state[i, 6])
                heading_uav = np.rad2deg(heading_uav)
            elif mode == 2:
                heading_uav = self.X_state[i, 5]
                heading_uav = np.rad2deg(heading_uav)
            if heading_uav > 0.0:
                heading_list.append(heading_uav - 180)
            else:
                heading_list.append(180 + heading_uav)

        self.heading_angle = np.array(heading_list)

    def disk(self, ax, x_ground, y_ground, pickup_height):
        R_disk = np.linspace(0, 0.2, 100)
        h_disk = np.array(
            [pickup_height - self.pickup_stick_height]).reshape(-1, 1)
        u_disk = np.linspace(0, 2 * np.pi, 100)
        x_disk = x_ground + np.outer(R_disk, np.cos(u_disk))
        y_disk = y_ground + np.outer(R_disk, np.sin(u_disk))
        draw_disk = ax.plot_surface(
            x_disk, y_disk, h_disk, color='gray', alpha=0.5)
        return draw_disk

    def showPath_3D(self, ):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.scatter(
            self.init_pose[0],
            self.init_pose[1],
            self.init_pose[2],
            color='r',
            marker='x',
        )
        for i in self.Wp_ref:
            ax.scatter(i[0], i[1], i[2], color='b', marker='x')

        ax.plot(self.X_state[:, 0], self.X_state[:, 1],
                self.X_state[:, 2], 'k-')
        ax.scatter(
            self.manipulator_states[:, 0], self.manipulator_states[:, 1], self.manipulator_states[:, 2], 'go')

        for i in range(self.num):
            if i in self.engage_index_list:
                ax.scatter(self.mobile_robot_states[i, 0], self.mobile_robot_states[i, 1],
                           self.pickup_point[2], color='r', marker='o')
            else:
                ax.scatter(self.mobile_robot_states[i, 0], self.mobile_robot_states[i, 1],
                           self.pickup_point[2], color='y', marker='o')

        # draw the head pointing of uav
        for i in range(self.num):
            eRb = self.rotation_matrix(self.X_state[i, 3:6])
            ax.plot(
                [self.X_state[i, 0], self.manipulator_states[i, 0]],
                [self.X_state[i, 1], self.manipulator_states[i, 1]],
                [self.X_state[i, 2], self.manipulator_states[i, 2]],
                'g-')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                    (eRb @ np.array([0.1, 0.0, 0.0]))[0]],
                [self.X_state[i, 1], self.X_state[i, 1] +
                    (eRb @ np.array([0.1, 0.0, 0.0]))[1]],
                [self.X_state[i, 2], self.X_state[i, 2] +
                    (eRb @ np.array([0.1, 0.0, 0.0]))[2]],
                color='r')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                    (eRb @ np.array([0.0, 0.1, 0.0]))[0]],
                [self.X_state[i, 1], self.X_state[i, 1] +
                    (eRb @ np.array([0.0, 0.1, 0.0]))[1]],
                [self.X_state[i, 2], self.X_state[i, 2] +
                    (eRb @ np.array([0.0, 0.1, 0.0]))[2]],
                color='y')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                    (eRb @ np.array([0.0, 0.0, 0.1]))[0]],
                [self.X_state[i, 1], self.X_state[i, 1] +
                    (eRb @ np.array([0.0, 0.0, 0.1]))[1]],
                [self.X_state[i, 2], self.X_state[i, 2] +
                    (eRb @ np.array([0.0, 0.0, 0.1]))[2]],
                color='b')
        ax.set_xlim([-1, 4])
        ax.set_ylim([-2.5, 2.5])
        ax.set_zlim([-1, 4])
        ax.set_xlabel("$x$")
        ax.set_ylabel("$y$")
        ax.set_zlabel("$z$")
        ax.set_title("3D UAV trajectory")

        plt.show()

    def showPath_XYplane(self, ):
        fig = plt.figure()
        ax = fig.gca()

        ax.scatter(
            self.init_pose[0],
            self.init_pose[1],
            color='r',
            marker='x',
        )
        for i in self.Wp_ref:
            ax.scatter(i[0], i[1], color='b', marker='x')
        ax.plot(self.X_state[:, 0], self.X_state[:, 1], 'k-')

        # draw the direction of mobile ground robot
        for i in range(self.num):
            if i in self.engage_index_list:
                ax.scatter(
                    self.mobile_robot_states[i, 0], self.mobile_robot_states[i, 1], color='r', marker='o')
                ax.plot(
                    [self.mobile_robot_states[i, 0], self.mobile_robot_states[i, 0] + 0.1 *
                        np.cos(np.arctan2(self.mobile_robot_states[i, 4], self.mobile_robot_states[i, 3]))],
                    [self.mobile_robot_states[i, 1], self.mobile_robot_states[i, 1] + 0.1 * np.sin(np.arctan2(self.mobile_robot_states[i, 4], self.mobile_robot_states[i, 3]))], 'r-')
            else:
                ax.scatter(
                    self.mobile_robot_states[i, 0], self.mobile_robot_states[i, 1], color='y', marker='o')

        for i in range(self.num):
            eRb = self.rotation_matrix(self.X_state[i, 3:6])
            ax.plot(
                [self.X_state[i, 0], self.manipulator_states[i, 0]],
                [self.X_state[i, 1], self.manipulator_states[i, 1]],
                'g-')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                   (eRb @ np.array([0.1, 0.0, 0.0]))[0]],
                [self.X_state[i, 1], self.X_state[i, 1] +
                    (eRb @ np.array([0.1, 0.0, 0.0]))[1]],
                'r-')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                    (eRb @ np.array([0.0, 0.1, 0.0]))[0]],
                [self.X_state[i, 1], self.X_state[i, 1] +
                    (eRb @ np.array([0.0, 0.1, 0.0]))[1]],
                'y-')
        ax.set_xlabel("$x$")
        ax.set_ylabel("$y$")
        ax.set_title('trajectory')
        plt.axis('equal')
        plt.show()

    def showPath_XZplane(self, ):
        fig = plt.figure()
        ax = fig.gca()

        v = np.ndarray((self.num, 1))
        for i in range(self.num):
            v[i, 0] = np.sqrt(self.X_state[i, 7]**2 +
                              self.X_state[i, 8]**2 + self.X_state[i, 9]**2)
        heatmap = plt.scatter(
            self.X_state[:, 0], self.X_state[:, 2], c=v, cmap=cm.rainbow, edgecolor='none', marker='o')
        cbar = plt.colorbar(heatmap, fraction=0.035)
        cbar.set_label("velocity of UAV [m/s]")

        ax.scatter(
            self.init_pose[0],
            self.init_pose[2],
            color='r',
            marker='x',
        )
        for i in self.Wp_ref:
            ax.scatter(i[0], i[2], color='b', marker='x')
        ax.plot(self.X_state[:, 0], self.X_state[:, 2], 'k-')

        # draw the trajectory of gripper
        ax.scatter(self.manipulator_states[:, 0],
                   self.manipulator_states[:, 2], color='g', marker='o')

        # draw the engage area
        for i in range(self.num):
            if i in self.engage_index_list:
                ax.scatter(
                    self.mobile_robot_states[i, 0], self.pickup_point[2], color='r', marker='o')
                theta = np.arange(0, 1 * np.pi, 0.01)
                x = self.mobile_robot_states[i, 0] + \
                    self.result_l_param[i] * np.cos(theta)
                z = self.pickup_point[2] + \
                    self.result_l_param[i] * np.sin(theta)
                ax.plot(x, z, 'r--')
                # print("the angle position of UAV inside engage area: \
                #     roll[{0}], pitch[{1}], yaw[{2}]".format(self.X_state[i,3],self.X_state[i,4],self.X_state[i,5]))
                print("vx_uav[{0}], vy_uav[{1}], vz_uav[{2}]\n".format(
                    self.X_state[i, 7], self.X_state[i, 8], self.X_state[i, 9]))
                print("vx_man[{0}], vy_man[{1}], vz_man[{2}]\n".format(
                    self.manipulator_states[i, 3], self.manipulator_states[i, 4], self.manipulator_states[i, 5]))
                print("vx_ground[{0}], vy_ground[{1}]\n".format(
                    self.mobile_robot_states[i, 3], self.mobile_robot_states[i, 4]))
            else:
                ax.scatter(
                    self.mobile_robot_states[i, 0], self.pickup_point[2], color='y', marker='o')

        for i in range(self.X_state[:, 3].shape[0]):
            eRb = self.rotation_matrix(self.X_state[i, 3:6])
            ax.plot(
                [self.X_state[i, 0], self.manipulator_states[i, 0]],
                [self.X_state[i, 2], self.manipulator_states[i, 2]],
                'g-')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                    (eRb @ np.array([0.1, 0.0, 0.0]))[0]],
                [self.X_state[i, 2], self.X_state[i, 2] +
                    (eRb @ np.array([0.1, 0.0, 0.0]))[2]],
                'b-')
            ax.plot(
                [self.X_state[i, 0], self.X_state[i, 0] +
                    (eRb @ np.array([-0.1, 0.0, 0.0]))[0]],
                [self.X_state[i, 2], self.X_state[i, 2] +
                    (eRb @ np.array([-0.1, 0.0, 0.0]))[2]],
                'b-')
        ax.set_xlabel("$x$")
        ax.set_ylabel("$z$")
        ax.set_title('trajectory')
        plt.legend(['UAV', 'Point area', ])
        plt.axis('equal')
        plt.show()

    def global_view_export(self, saveGIF=False, saveMP4=False, export_name='uav_global', fps=10):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        # draw initial and waypoints
        ax.scatter(self.init_pose[0], self.init_pose[1],
                   self.init_pose[2], color='r', marker='x',)
        for i in self.Wp_ref:
            ax.scatter(i[0], i[1], i[2], color='b', marker='x')

        # draw curve
        x_uav = self.X_state[:, 0]
        y_uav = self.X_state[:, 1]
        z_uav = self.X_state[:, 2]
        ax.plot(x_uav, y_uav, z_uav, 'k-')
        ax.plot(self.mobile_robot_states[:, 0].flatten(),
                self.mobile_robot_states[:, 1].flatten(), self.mobile_robot_states[:, 2].flatten(), 'r-')

        ax.view_init(azim=-120, elev=20)
        ax.grid(False)
        # ax.autoscale_view()
        ax.set_box_aspect((np.ptp(x_uav), np.ptp(y_uav), np.ptp(z_uav)))
        time_text = ax.text(0, 1, 1, '',
                            zdir=(0.5, 0, 0))  # transform=ax.transAxes

        # create list for plotting
        self.draw_x_axis = []
        self.draw_y_axis = []
        self.draw_z_axis = []
        self.draw_manipulator = []
        self.draw_endmanip = []

        leng_heading = 0.3
        Frames = 4

        def animation_init():
            time_text.set_text('')
            eRb = self.rotation_matrix(self.X_state[0, 3:6])
            # draw frame of uav
            self.draw_x_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[2]),
                fc='red', ec='red', arrowstyle="-|>", mutation_scale=15))
            self.draw_y_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[2]),
                fc='brown', ec='brown', arrowstyle="-|>", mutation_scale=15))
            self.draw_z_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[2]),
                fc='blue', ec='blue', arrowstyle="-|>", mutation_scale=15))
            self.draw_manipulator.append(Arrow3D(
                (self.X_state[0, 0], self.manipulator_states[0, 0]),
                (self.X_state[0, 1], self.manipulator_states[0, 1]),
                (self.X_state[0, 2], self.manipulator_states[0, 2]),
                fc='green', ec='green', arrowstyle="-", mutation_scale=30))
            ax.add_artist(self.draw_x_axis[0])
            ax.add_artist(self.draw_y_axis[0])
            ax.add_artist(self.draw_z_axis[0])
            ax.add_artist(self.draw_manipulator[0])

            self.draw_disk = self.disk(
                ax, self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 1], self.mobile_robot_states[0, 2])
            self.draw_amr = ax.scatter(
                self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 1], self.mobile_robot_states[0, 2], s=25, color='red')
            self.draw_link = ax.plot([self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 0]],
                                     [self.mobile_robot_states[0, 1],
                                         self.mobile_robot_states[0, 1]],
                                     [self.pickup_point[2],
                                         self.pickup_point[2] - self.pickup_stick_height],
                                     'r-', linewidth=3)

            self.draw_endmanip.append(ax.scatter(
                self.manipulator_states[0, 0], self.manipulator_states[0, 1], self.manipulator_states[0, 2], s=25, color='blue'))

            return self.draw_x_axis, self.draw_y_axis, self.draw_z_axis,

        def animation_loop(index):
            time_text.set_text('time = %.2f' % self.time_stamps[index])
            eRb = self.rotation_matrix(self.X_state[index, 3:6])

            if len(self.draw_x_axis) > Frames:
                self.draw_x_axis[0].remove()
                self.draw_y_axis[0].remove()
                self.draw_z_axis[0].remove()
                self.draw_manipulator[0].remove()
                self.draw_endmanip[0].remove()
                self.draw_x_axis = self.draw_x_axis[1:]
                self.draw_y_axis = self.draw_y_axis[1:]
                self.draw_z_axis = self.draw_z_axis[1:]
                self.draw_manipulator = self.draw_manipulator[1:]
                self.draw_endmanip = self.draw_endmanip[1:]

            self.draw_amr.remove()
            self.draw_link.pop(0).remove()
            self.draw_disk.remove()

            self.draw_x_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[2]),
                fc='red', ec='red', arrowstyle="-|>", mutation_scale=15,))
            self.draw_y_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[2]),
                fc='brown', ec='brown', arrowstyle="-|>", mutation_scale=15))
            self.draw_z_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[2]),
                fc='blue', ec='blue', arrowstyle="-|>", mutation_scale=15))
            self.draw_manipulator.append(Arrow3D(
                (self.X_state[index, 0], self.manipulator_states[index, 0]),
                (self.X_state[index, 1], self.manipulator_states[index, 1]),
                (self.X_state[index, 2], self.manipulator_states[index, 2]),
                fc='green', ec='green', arrowstyle="-", mutation_scale=30))
            ax.add_artist(self.draw_x_axis[-1])
            ax.add_artist(self.draw_y_axis[-1])
            ax.add_artist(self.draw_z_axis[-1])
            ax.add_artist(self.draw_manipulator[-1])

            self.draw_disk = self.disk(ax,
                                       self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1], self.mobile_robot_states[index, 2])
            self.draw_amr = ax.scatter(
                self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1], self.mobile_robot_states[index, 2], s=25, color='red')
            self.draw_link = ax.plot([self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 0]],
                                     [self.mobile_robot_states[index, 1],
                                      self.mobile_robot_states[index, 1]],
                                     [self.mobile_robot_states[index, 2],
                                         self.mobile_robot_states[index, 2] - self.pickup_stick_height],
                                     'r-', linewidth=3)

            self.draw_endmanip.append(ax.scatter(
                self.manipulator_states[index, 0], self.manipulator_states[index, 1], self.manipulator_states[index, 2], s=25, color='blue'))

            if len(self.draw_x_axis) > Frames:
                for i in range(5):
                    self.draw_x_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_y_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_z_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_manipulator[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_endmanip[i].set_alpha(0.04 + 0.24 * i)

            if index == self.N:
                for i in range(4):
                    self.draw_x_axis[i].remove()
                    self.draw_y_axis[i].remove()
                    self.draw_z_axis[i].remove()
                    self.draw_manipulator[i].remove()
                    self.draw_endmanip[i].remove()

            return self.draw_x_axis, self.draw_y_axis, self.draw_z_axis,\
                self.draw_manipulator

        ani = animation.FuncAnimation(fig, animation_loop, range(
            self.N + 1), init_func=animation_init, interval=100, repeat=False)
        plt.grid('--')
        if saveGIF:
            # ani.save('./' + export_name + '.gif',
            #          writer='imagemagick', fps=fps, dpi=400)
            ani.save('./' + export_name + '.gif',
                     writer=animation.FFMpegWriter(fps=fps), dpi=400)
        elif saveMP4:
            print('saving mp4')
            # ani.save('./' + export_name + '.mp4',
            #          writer='imagemagick', fps=fps, dpi=400)
            ani.save('./' + export_name + '.mp4',
                     writer=animation.FFMpegWriter(fps=fps), dpi=400)
        if not saveGIF and not saveMP4:
            plt.gcf().set_dpi(300)
            plt.show()

    def following_view_export(self, lock_target=False, saveGIF=False, saveMP4=False, export_name='uav_following_view', fps=10):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # draw pin set
        ax.scatter(self.init_pose[0], self.init_pose[1],
                   self.init_pose[2], color='r', marker='x',)
        for i in self.Wp_ref:
            ax.scatter(i[0], i[1], i[2], color='b', marker='x')

        # draw curve
        x_uav = self.X_state[:, 0]
        y_uav = self.X_state[:, 1]
        z_uav = self.X_state[:, 2]
        ax.plot(x_uav, y_uav, z_uav, 'k-')
        ax.plot(self.mobile_robot_states[:, 0].flatten(),
                self.mobile_robot_states[:, 1].flatten(), self.mobile_robot_states[:, 2].flatten(), 'r-')

        ax.view_init(azim=-180, elev=10)
        ax.grid(False)
        # ax.autoscale_view()
        # ax.set_box_aspect((np.ptp(x_uav), np.ptp(y_uav), np.ptp(z_uav)))
        time_text = ax.text(0, 1, 1, '',
                            zdir=(0.5, 0, 0))  # transform=ax.transAxes
        # create list for plotting
        self.draw_x_axis = []
        self.draw_y_axis = []
        self.draw_z_axis = []
        self.draw_manipulator = []
        self.draw_endmanip = []

        leng_heading = 0.3
        Frames = 4

        self.cal_heading()

        def animation_init():
            time_text.set_text('')

            eRb = self.rotation_matrix(self.X_state[0, 3:6])
            # draw frame of uav
            self.draw_x_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[2]),
                fc='red', ec='red', arrowstyle="-|>", mutation_scale=15))
            self.draw_y_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[2]),
                fc='brown', ec='brown', arrowstyle="-|>", mutation_scale=15))
            self.draw_z_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[2]),
                fc='blue', ec='blue', arrowstyle="-|>", mutation_scale=15))
            self.draw_manipulator.append(Arrow3D(
                (self.X_state[0, 0], self.manipulator_states[0, 0]),
                (self.X_state[0, 1], self.manipulator_states[0, 1]),
                (self.X_state[0, 2], self.manipulator_states[0, 2]),
                fc='green', ec='green', arrowstyle="-", mutation_scale=30))
            ax.add_artist(self.draw_x_axis[0])
            ax.add_artist(self.draw_y_axis[0])
            ax.add_artist(self.draw_z_axis[0])
            ax.add_artist(self.draw_manipulator[0])

            self.draw_disk = self.disk(
                ax, self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 1], self.mobile_robot_states[0, 2])
            self.draw_amr = ax.scatter(
                self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 1], self.mobile_robot_states[0, 2], s=25, color='red')
            self.draw_link = ax.plot([self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 0]],
                                     [self.mobile_robot_states[0, 1],
                                         self.mobile_robot_states[0, 1]],
                                     [self.mobile_robot_states[0, 2],
                                         self.mobile_robot_states[0, 2] - self.pickup_stick_height],
                                     'r-', linewidth=3)

            self.draw_endmanip.append(ax.scatter(
                self.manipulator_states[0, 0], self.manipulator_states[0, 1], self.manipulator_states[0, 2], s=25, color='blue'))

            return self.draw_x_axis, self.draw_y_axis, self.draw_z_axis,

        def animation_loop(index):
            time_text.set_text('time = %.2f' % self.time_stamps[index])

            eRb = self.rotation_matrix(self.X_state[index, 3:6])
            # view of heading
            if lock_target:
                ax.view_init(azim=self.heading_angle.flatten()[
                             index], elev=self.X_state[index, 4] + 10)
                ax.set_xlim3d(self.X_state[index, 0] - 0.,
                              self.X_state[index, 0] + 0.4)
                ax.set_ylim3d(
                    self.X_state[index, 1] - 0.4, self.X_state[index, 1] + 0.4)
                ax.set_zlim3d(self.X_state[index, 2] -
                              0.3, self.X_state[index, 2] + 0.3)

            if len(self.draw_x_axis) > Frames:
                self.draw_x_axis[0].remove()
                self.draw_y_axis[0].remove()
                self.draw_z_axis[0].remove()
                self.draw_manipulator[0].remove()
                self.draw_endmanip[0].remove()
                self.draw_x_axis = self.draw_x_axis[1:]
                self.draw_y_axis = self.draw_y_axis[1:]
                self.draw_z_axis = self.draw_z_axis[1:]
                self.draw_manipulator = self.draw_manipulator[1:]
                self.draw_endmanip = self.draw_endmanip[1:]

            self.draw_amr.remove()
            self.draw_link.pop(0).remove()
            self.draw_disk.remove()

            self.draw_x_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[2]),
                fc='red', ec='red', arrowstyle="-|>", mutation_scale=15,))
            self.draw_y_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[2]),
                fc='brown', ec='brown', arrowstyle="-|>", mutation_scale=15))
            self.draw_z_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[2]),
                fc='blue', ec='blue', arrowstyle="-|>", mutation_scale=15))
            self.draw_manipulator.append(Arrow3D(
                (self.X_state[index, 0], self.manipulator_states[index, 0]),
                (self.X_state[index, 1], self.manipulator_states[index, 1]),
                (self.X_state[index, 2], self.manipulator_states[index, 2]),
                fc='green', ec='green', arrowstyle="-", mutation_scale=30))
            ax.add_artist(self.draw_x_axis[-1])
            ax.add_artist(self.draw_y_axis[-1])
            ax.add_artist(self.draw_z_axis[-1])
            ax.add_artist(self.draw_manipulator[-1])

            self.draw_disk = self.disk(ax,
                                       self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1], self.mobile_robot_states[index, 2])
            self.draw_amr = ax.scatter(
                self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1], self.mobile_robot_states[index, 2], s=25, color='red')
            self.draw_link = ax.plot([self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 0]],
                                     [self.mobile_robot_states[index, 1],
                                      self.mobile_robot_states[index, 1]],
                                     [self.mobile_robot_states[index, 2],
                                         self.mobile_robot_states[index, 2] - self.pickup_stick_height],
                                     'r-', linewidth=3)

            self.draw_endmanip.append(ax.scatter(
                self.manipulator_states[index, 0], self.manipulator_states[index, 1], self.manipulator_states[index, 2], s=25, color='blue'))

            if len(self.draw_x_axis) > Frames:
                for i in range(5):
                    self.draw_x_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_y_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_z_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_manipulator[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_endmanip[i].set_alpha(0.04 + 0.24 * i)

            if index == self.N:
                for i in range(4):
                    self.draw_x_axis[i].remove()
                    self.draw_y_axis[i].remove()
                    self.draw_z_axis[i].remove()
                    self.draw_manipulator[i].remove()
                    self.draw_endmanip[i].remove()

            return self.draw_x_axis, self.draw_y_axis, self.draw_z_axis,\
                self.draw_manipulator

        ani = animation.FuncAnimation(fig, animation_loop, range(
            self.N + 1), init_func=animation_init, interval=100, repeat=False)
        plt.grid('--')
        if saveGIF:
            # ani.save('./' + export_name + '.gif',
            #          writer='imagemagick', fps=fps, dpi=400)
            ani.save('./' + export_name + '.gif',
                     writer=animation.FFMpegWriter(fps=fps), dpi=400)
        elif saveMP4:
            print('saving mp4')
            # ani.save('./' + export_name + '.mp4',
            #          writer='imagemagick', fps=fps, dpi=400)
            ani.save('./' + export_name + '.mp4',
                     writer=animation.FFMpegWriter(fps=fps), dpi=400)
        if not saveGIF and not saveMP4:
            plt.gcf().set_dpi(300)
            plt.show()

    def top_view_export(self, lock_target=False, saveGIF=False, saveMP4=False, export_name='uav_top_view', fps=10):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # draw pin set
        ax.scatter(self.init_pose[0], self.init_pose[1],
                   self.init_pose[2], color='r', marker='x',)
        for i in self.Wp_ref:
            ax.scatter(i[0], i[1], i[2], color='b', marker='x')

        # draw curve
        x_uav = self.X_state[:, 0]
        y_uav = self.X_state[:, 1]
        z_uav = self.X_state[:, 2]
        ax.plot(x_uav, y_uav, z_uav, 'k-')
        ax.plot(self.mobile_robot_states[:, 0].flatten(),
                self.mobile_robot_states[:, 1].flatten(), self.mobile_robot_states[:, 2].flatten(), 'r-')

        ax.view_init(azim=-180, elev=90)
        ax.grid(False)
        # ax.autoscale_view()
        # ax.set_box_aspect((np.ptp(x_uav), np.ptp(y_uav), np.ptp(z_uav)))
        time_text = ax.text(0, 1, 1, '',
                            zdir=(0.5, 0, 0))  # transform=ax.transAxes
        # create list for plotting
        self.draw_x_axis = []
        self.draw_y_axis = []
        self.draw_z_axis = []
        self.draw_manipulator = []
        self.draw_endmanip = []

        leng_heading = 0.3
        Frames = 4

        def animation_init():
            time_text.set_text('')

            eRb = self.rotation_matrix(self.X_state[0, 3:6])
            # draw frame of uav
            self.draw_x_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[2]),
                fc='red', ec='red', arrowstyle="-|>", mutation_scale=15))
            self.draw_y_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[2]),
                fc='brown', ec='brown', arrowstyle="-|>", mutation_scale=15))
            self.draw_z_axis.append(Arrow3D(
                (self.X_state[0, 0], self.X_state[0, 0] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[0]),
                (self.X_state[0, 1], self.X_state[0, 1] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[1]),
                (self.X_state[0, 2], self.X_state[0, 2] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[2]),
                fc='blue', ec='blue', arrowstyle="-|>", mutation_scale=15))
            self.draw_manipulator.append(Arrow3D(
                (self.X_state[0, 0], self.manipulator_states[0, 0]),
                (self.X_state[0, 1], self.manipulator_states[0, 1]),
                (self.X_state[0, 2], self.manipulator_states[0, 2]),
                fc='green', ec='green', arrowstyle="-", mutation_scale=30))
            ax.add_artist(self.draw_x_axis[0])
            ax.add_artist(self.draw_y_axis[0])
            ax.add_artist(self.draw_z_axis[0])
            ax.add_artist(self.draw_manipulator[0])

            self.draw_disk = self.disk(
                ax, self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 1], self.mobile_robot_states[0, 2])
            self.draw_amr = ax.scatter(
                self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 1], self.mobile_robot_states[0, 2], s=25, color='red')
            self.draw_link = ax.plot([self.mobile_robot_states[0, 0], self.mobile_robot_states[0, 0]],
                                     [self.mobile_robot_states[0, 1],
                                         self.mobile_robot_states[0, 1]],
                                     [self.mobile_robot_states[0, 2],
                                         self.mobile_robot_states[0, 2] - self.pickup_stick_height],
                                     'r-', linewidth=3)

            self.draw_endmanip.append(ax.scatter(
                self.manipulator_states[0, 0], self.manipulator_states[0, 1], self.manipulator_states[0, 2], s=25, color='blue'))

            return self.draw_x_axis, self.draw_y_axis, self.draw_z_axis,

        def animation_loop(index):
            time_text.set_text('time = %.2f' % self.time_stamps[index])

            # time_text.set_position(
            #     (self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1]))
            # time_text.update()
            eRb = self.rotation_matrix(self.X_state[index, 3:6])

            # view of heading
            if lock_target:
                # ax.view_init(azim = cal_heading()[index], elev = 20)
                ax.set_xlim3d(self.mobile_robot_states[index, 0] - 0.4,
                              self.mobile_robot_states[index, 0] + 0.4)
                ax.set_ylim3d(
                    self.mobile_robot_states[index, 1] - 0.4, self.mobile_robot_states[index, 1] + 0.4)
                ax.set_zlim3d(0, self.mobile_robot_states[index, 2] + 0.3)

            if len(self.draw_x_axis) > Frames:
                self.draw_x_axis[0].remove()
                self.draw_y_axis[0].remove()
                self.draw_z_axis[0].remove()
                self.draw_manipulator[0].remove()
                self.draw_endmanip[0].remove()
                self.draw_x_axis = self.draw_x_axis[1:]
                self.draw_y_axis = self.draw_y_axis[1:]
                self.draw_z_axis = self.draw_z_axis[1:]
                self.draw_manipulator = self.draw_manipulator[1:]
                self.draw_endmanip = self.draw_endmanip[1:]

            self.draw_amr.remove()
            self.draw_link.pop(0).remove()
            self.draw_disk.remove()

            self.draw_x_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([leng_heading, 0.0, 0.0]))[2]),
                fc='red', ec='red', arrowstyle="-|>", mutation_scale=15,))
            self.draw_y_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([0.0, leng_heading, 0.0]))[2]),
                fc='brown', ec='brown', arrowstyle="-|>", mutation_scale=15))
            self.draw_z_axis.append(Arrow3D(
                (self.X_state[index, 0], self.X_state[index, 0] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[0]),
                (self.X_state[index, 1], self.X_state[index, 1] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[1]),
                (self.X_state[index, 2], self.X_state[index, 2] +
                 (eRb @ np.array([0.0, 0.0, leng_heading]))[2]),
                fc='blue', ec='blue', arrowstyle="-|>", mutation_scale=15))
            self.draw_manipulator.append(Arrow3D(
                (self.X_state[index, 0], self.manipulator_states[index, 0]),
                (self.X_state[index, 1], self.manipulator_states[index, 1]),
                (self.X_state[index, 2], self.manipulator_states[index, 2]),
                fc='green', ec='green', arrowstyle="-", mutation_scale=30))
            ax.add_artist(self.draw_x_axis[-1])
            ax.add_artist(self.draw_y_axis[-1])
            ax.add_artist(self.draw_z_axis[-1])
            ax.add_artist(self.draw_manipulator[-1])

            self.draw_disk = self.disk(ax,
                                       self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1], self.mobile_robot_states[index, 2])
            self.draw_amr = ax.scatter(
                self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 1], self.mobile_robot_states[index, 2], s=25, color='red')
            self.draw_link = ax.plot([self.mobile_robot_states[index, 0], self.mobile_robot_states[index, 0]],
                                     [self.mobile_robot_states[index, 1],
                                      self.mobile_robot_states[index, 1]],
                                     [self.mobile_robot_states[index, 2],
                                         self.mobile_robot_states[index, 2] - self.pickup_stick_height],
                                     'r-', linewidth=3)

            self.draw_endmanip.append(ax.scatter(
                self.manipulator_states[index, 0], self.manipulator_states[index, 1], self.manipulator_states[index, 2], s=25, color='blue'))

            if len(self.draw_x_axis) > Frames:
                for i in range(5):
                    self.draw_x_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_y_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_z_axis[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_manipulator[i].set_alpha(0.04 + 0.24 * i)
                    self.draw_endmanip[i].set_alpha(0.04 + 0.24 * i)

            if index == self.N:
                for i in range(4):
                    self.draw_x_axis[i].remove()
                    self.draw_y_axis[i].remove()
                    self.draw_z_axis[i].remove()
                    self.draw_manipulator[i].remove()
                    self.draw_endmanip[i].remove()

            return self.draw_x_axis, self.draw_y_axis, self.draw_z_axis,\
                self.draw_manipulator

        ani = animation.FuncAnimation(fig, animation_loop, range(
            self.N + 1), init_func=animation_init, interval=100, repeat=False)
        plt.grid('--')
        if saveGIF:
            # ani.save('./' + export_name + '.gif',
            #          writer='imagemagick', fps=fps, dpi=400)
            ani.save('./' + export_name + '.gif',
                     writer=animation.FFMpegWriter(fps=fps), dpi=400)
        elif saveMP4:
            print('saving mp4')
            # ani.save('./' + export_name + '.mp4',
            #          writer='imagemagick', fps=fps, dpi=400)
            ani.save('./' + export_name + '.mp4',
                     writer=animation.FFMpegWriter(fps=fps), dpi=400)
        if not saveGIF and not saveMP4:
            plt.gcf().set_dpi(300)
            plt.show()


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super(Arrow3D, self).__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

# 2D result plot class


class show2DResults(object):
    def __init__(self, Tn, N, uav_states, engage_index, mobile_robot_states, manipulator_states, pickup_point, with_moving_manipulator=True):
        self.Tn = Tn
        self.N = N
        self.time_stamps = np.linspace(0, Tn, N + 1, endpoint=True)
        self.X_state = uav_states  # in shape [index, state]
        self.dt = Tn / N
        start_time = self.dt * engage_index[0, 0]
        self.start_time = start_time.item()
        end_time = self.dt * engage_index[0, -1]
        self.end_time = end_time.item()
        self.time_interval = end_time - start_time
        self.mobile_robot_states = mobile_robot_states
        self.manipulator_states = manipulator_states
        self.pickup_point = pickup_point
        self.with_moving_manipulator = with_moving_manipulator

        # make up the data for ground robot
        self.mobile_robot_states = np.hstack((self.mobile_robot_states[:, :2], self.pickup_point[2] * np.ones(
            (self.mobile_robot_states.shape[0], 1)), self.mobile_robot_states[:, 2:], np.zeros((self.mobile_robot_states.shape[0], 1))))

        # find the Overlap Point to determine the real collision time
        overlap_index = 0
        overlap_index_list = []
        self.dis_ = []  # distance between the endposition of manipulator and the autonomous mobile robot
        for i in range(self.mobile_robot_states.shape[0]):
            array_temp = self.manipulator_states[i, :3] - np.concatenate(
                (self.mobile_robot_states[i, :2], [self.pickup_point[2]])).reshape(1, -1)
            dis_temp = np.linalg.norm(array_temp, ord=2)
            self.dis_.append(dis_temp)
            if dis_temp <= 0.01:
                overlap_index_list.append(overlap_index)
            overlap_index += 1

        # calculate the real collision time for plotting
        overlap_time = []
        for i in overlap_index_list:
            overlap_time.append([self.dt * i, self.dt * (i + 1)])
        self.overlap_time = np.array(overlap_time).reshape(-1, 2)

    def plot_uav_position_velocity(self, savePNG=False, saveSVG=False, dpi=400):
        fig, axs = plt.subplots(3, 2, sharex='col',
                                figsize=(19 / 2.54, 10 / 2.54))  # use /2.54 to convert from cm to inches
        fig.suptitle("UAV positions and velocities")
        axs[0, 0].plot(self.time_stamps, self.X_state[:, 0].flatten(),
                       marker="o", markersize=4)
        # axs[0, 0].set_title('$x$(uav)')
        axs[0, 0].set_ylabel('$x$ [m]')
        axs[1, 0].plot(self.time_stamps, self.X_state[:,
                       1].flatten(), marker="o", markersize=4)
        # axs[1, 0].set_title('$y$(uav)')
        axs[1, 0].set_ylabel('$y$ [m]')
        axs[2, 0].plot(self.time_stamps, self.X_state[:,
                       2].flatten(), marker="o", markersize=4)
        # axs[2, 0].set_title('$z$(uav)')
        axs[2, 0].set_ylabel('$z$ [m]')
        axs[2, 0].set_xlabel('time [s]')

        if self.with_moving_manipulator:
            axs[0, 1].plot(self.time_stamps, self.X_state[:,
                           7].flatten(), marker="o", markersize=4)
            # axs[0, 1].set_title('$v_x$(uav)')
            axs[0, 1].set_ylabel('$v_x$ [m/s]')
            axs[1, 1].plot(self.time_stamps, self.X_state[:,
                           8].flatten(), marker="o", markersize=4)
            # axs[1, 1].set_title('vy(uav)')
            axs[1, 1].set_ylabel('$v_y$ [m/s]')
            axs[2, 1].plot(self.time_stamps, self.X_state[:,
                           9].flatten(), marker="o", markersize=4)
            # axs[2, 1].set_title('vz(uav)')
            axs[2, 1].set_ylabel('$v_z$ [m/s]')
            axs[2, 1].set_xlabel('time [s]')
        else:
            axs[0, 1].plot(self.time_stamps, self.X_state[:,
                           6].flatten(), marker="o", markersize=4)
            # axs[0, 1].set_title('$v_x$(uav)')
            axs[0, 1].set_ylabel('$v_x$ [m/s]')
            axs[1, 1].plot(self.time_stamps, self.X_state[:,
                           7].flatten(), marker="o", markersize=4)
            # axs[1, 1].set_title('vy(uav)')
            axs[1, 1].set_ylabel('$v_y$ [m/s]')
            axs[2, 1].plot(self.time_stamps, self.X_state[:,
                           8].flatten(), marker="o", markersize=4)
            # axs[2, 1].set_title('vz(uav)')
            axs[2, 1].set_ylabel('$v_z$ [m/s]')
            axs[2, 1].set_xlabel('time [s]')
        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                [ymin, ymax] = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(patches.Rectangle((
                    self.start_time, ymin),
                    self.time_interval,  # width
                    rec_height,  # height
                    edgecolor='None',
                    facecolor='mistyrose',
                    fill=True
                ))
                for k in self.overlap_time:
                    axs[i, j].add_patch(patches.Rectangle(
                        (k[0], ymin),
                        self.dt,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='cyan',
                        fill=True
                    ))

        if savePNG:
            plt.savefig('uav_position_velocity.png', format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('uav_position_velocity.svg', format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_uav_attitude(self, savePNG=False, saveSVG=False, dpi=400):
        if self.with_moving_manipulator:
            fig, axs = plt.subplots(4, 2, sharex='col',
                                    figsize=(19 / 2.54, 10 / 2.54))  # use /2.54 to convert from cm to inches
            fig.suptitle("UAV attitudes")
            axs[0, 0].plot(self.time_stamps, self.X_state[:,
                                                          3].flatten(), marker="o", markersize=4)
            # axs[0, 0].set_title('roll angle of uav')
            axs[0, 0].set_ylabel('$\phi$ [rad]')
            axs[1, 0].plot(self.time_stamps, self.X_state[:,
                                                          4].flatten(), marker="o", markersize=4)
            # axs[1, 0].set_title('pitch angle of uav')
            axs[1, 0].set_ylabel(r'$\theta$ [rad]')
            axs[2, 0].plot(self.time_stamps, self.X_state[:,
                                                          5].flatten(), marker="o", markersize=4)
            # axs[2, 0].set_title('yaw angle of uav')
            axs[2, 0].set_ylabel('$\psi$ [rad]')
            axs[3, 0].plot(self.time_stamps, self.X_state[:,
                           6].flatten(), marker="o", markersize=4)
            # axs[3, 0].set_title('rotation angle of manipulator')
            axs[3, 0].set_ylabel(r'$\alpha$ [rad]')
            axs[3, 0].set_xlabel('time [s]')
            axs[0, 1].plot(self.time_stamps, self.X_state[:,
                           10].flatten(), marker="o", markersize=4)
            # axs[0, 1].set_title('roll angle velocity')
            axs[0, 1].set_ylabel('d$\phi$ [rad/s]')
            axs[1, 1].plot(self.time_stamps, self.X_state[:,
                           11].flatten(), marker="o", markersize=4)
            # axs[1, 1].set_title('pitch angle velocity')
            axs[1, 1].set_ylabel(r'd$\theta$ [rad/s]')
            axs[2, 1].plot(self.time_stamps, self.X_state[:,
                           12].flatten(), marker="o", markersize=4)
            # axs[2, 1].set_title('yaw angle velocity')
            axs[2, 1].set_ylabel('d$\psi$ [rad/s]')
            axs[3, 1].plot(self.time_stamps, self.X_state[:,
                           13].flatten(), marker="o", markersize=4)
            # axs[3, 1].set_title('rotation velocity of manipulator')
            axs[3, 1].set_ylabel(r'd$\alpha$ [rad/s]')
            axs[3, 1].set_xlabel('time [s]')
        else:
            fig, axs = plt.subplots(3, 2, sharex='col',
                                    figsize=(19 / 2.54, 10 / 2.54))  # use /2.54 to convert from cm to inches
            fig.suptitle("UAV attitudes")
            axs[0, 0].plot(self.time_stamps, self.X_state[:,
                           3].flatten(), marker="o", markersize=4)
            # axs[0, 0].set_title('roll angle of uav')
            axs[0, 0].set_ylabel('$\phi$ [rad]')
            axs[1, 0].plot(self.time_stamps, self.X_state[:,
                           4].flatten(), marker="o", markersize=4)
            # axs[1, 0].set_title('pitch angle of uav')
            axs[1, 0].set_ylabel(r'$\theta$ [rad]')
            axs[2, 0].plot(self.time_stamps, self.X_state[:,
                           5].flatten(), marker="o", markersize=4)
            # axs[2, 0].set_title('yaw angle of uav')
            axs[2, 0].set_ylabel('$\psi$ [rad]')
            axs[0, 1].plot(self.time_stamps, self.X_state[:,
                           9].flatten(), marker="o", markersize=4)
            # axs[0, 1].set_title('roll angle velocity')
            axs[0, 1].set_ylabel('d$\phi$ [rad/s]')
            axs[1, 1].plot(self.time_stamps, self.X_state[:,
                           10].flatten(), marker="o", markersize=4)
            # axs[1, 1].set_title('pitch angle velocity')
            axs[1, 1].set_ylabel(r'd$\theta$ [rad/s]')
            axs[2, 1].plot(self.time_stamps, self.X_state[:,
                           11].flatten(), marker="o", markersize=4)
            # axs[2, 1].set_title('yaw angle velocity')
            axs[2, 1].set_ylabel('d$\psi$ [rad/s]')
        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                ymin, ymax = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(
                    patches.Rectangle(
                        (self.start_time, ymin),
                        self.time_interval,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='mistyrose',
                        fill=True
                    ))
                for _ in self.overlap_time:
                    axs[i, j].add_patch(
                        patches.Rectangle(
                            (_[0], ymin),
                            self.dt,  # width
                            rec_height,  # height
                            edgecolor='None',
                            facecolor='cyan',
                            fill=True
                        ))
        if savePNG:
            plt.savefig('uav_attitude.png', format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('uav_attitude.svg', format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_manipulator_state(self, savePNG=False, saveSVG=False, dpi=400):
        fig, axs = plt.subplots(3, 2, sharex='col',
                                figsize=(19 / 2.54, 10 / 2.54))  # use /2.54 to convert from cm to inches
        fig.suptitle("manipulator states")
        axs[0, 0].plot(self.time_stamps, self.manipulator_states[:,
                       0].flatten(), marker="o", markersize=4)
        # axs[0, 0].set_title('$x$ (manipulator)')
        axs[0, 0].set_ylabel('$x$ [m]')
        axs[1, 0].plot(self.time_stamps, self.manipulator_states[:,
                       1].flatten(), marker="o", markersize=4)
        # axs[1, 0].set_title('$y$ (manipulator)')
        axs[1, 0].set_ylabel('$y$ [m]')
        axs[2, 0].plot(self.time_stamps, self.manipulator_states[:,
                       2].flatten(), marker="o", markersize=4)
        # axs[2, 0].set_title('$z$ (manipulator)')
        axs[2, 0].set_ylabel('$z$ [m]')
        axs[2, 0].set_xlabel('time [s]')

        axs[0, 1].plot(self.time_stamps, self.manipulator_states[:,
                       3].flatten(), marker="o", markersize=4)
        # axs[0, 1].set_title('$v_x$ (manipulator)')
        axs[0, 1].set_ylabel('$v_x$ [m/s]')
        axs[1, 1].plot(self.time_stamps, self.manipulator_states[:,
                       4].flatten(), marker="o", markersize=4)
        # axs[1, 1].set_title('$v_y$ (manipulator)')
        axs[1, 1].set_ylabel('$v_y$ [m/s]')
        axs[2, 1].plot(self.time_stamps, self.manipulator_states[:,
                       5].flatten(), marker="o", markersize=4)
        # axs[2, 1].set_title('$v_z$ (manipulator)')
        axs[2, 1].set_ylabel('$v_z$ [m/s]')
        axs[2, 1].set_xlabel('time [s]')
        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                ymin, ymax = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(
                    patches.Rectangle(
                        (self.start_time, ymin),
                        self.time_interval,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='mistyrose',
                        fill=True
                    ))
                for _ in self.overlap_time:
                    axs[i, j].add_patch(
                        patches.Rectangle(
                            (_[0], ymin),
                            self.dt,  # width
                            rec_height,  # height
                            edgecolor='None',
                            facecolor='cyan',
                            fill=True
                        ))

        if savePNG:
            plt.savefig('manipulator_state.png', format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('manipulator_state.svg', format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_target_robot_state(self, savePNG=False, saveSVG=False, dpi=400):
        fig, axs = plt.subplots(2, 2, sharex='col',
                                figsize=(19 / 2.54, 10 / 2.54))
        fig.suptitle("target robot states")
        axs[0, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                       0].flatten(), marker="o", markersize=4)
        # axs[0, 0].set_title('x autonomous mobile robot(AMR)')
        axs[0, 0].set_ylabel('$x$ [m]')
        axs[1, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                       1].flatten(), marker="o", markersize=4)
        # axs[1, 0].set_title('y autonomous mobile robot(AMR)')
        axs[1, 0].set_ylabel('$y$ [m]')
        axs[1, 0].set_xlabel('time [s]')

        axs[0, 1].plot(self.time_stamps, self.mobile_robot_states[:,
                       3].flatten(), marker="o", markersize=4)
        # axs[0, 1].set_title('vx autonomous mobile robot(AMR)')
        axs[0, 1].set_ylabel('$v_x$ [m/s]')
        axs[1, 1].plot(self.time_stamps, self.mobile_robot_states[:,
                       4].flatten(), marker="o", markersize=4)
        # axs[1, 1].set_title('vy autonomous mobile robot(AMR)')
        axs[1, 1].set_ylabel('$v_y$ [m/s]')
        axs[1, 1].set_xlabel('time [s]')
        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                ymin, ymax = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(
                    patches.Rectangle(
                        (self.start_time, ymin),
                        self.time_interval,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='mistyrose',
                        fill=True
                    ))
                for _ in self.overlap_time:
                    axs[i, j].add_patch(
                        patches.Rectangle(
                            (_[0], ymin),
                            self.dt,  # width
                            rec_height,  # height
                            edgecolor='None',
                            facecolor='cyan',
                            fill=True
                        ))
        if savePNG:
            plt.savefig('target_robot_state.png', format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('target_robot_state.svg', format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_uav_robot_comparison(self, savePNG=False, saveSVG=False, dpi=400):
        fig, axs = plt.subplots(2, 2, sharex='col',
                                figsize=(19 / 2.54, 10 / 2.54))
        fig.suptitle("UAV and AMR velocities comparision")
        if self.with_moving_manipulator:
            axs[0, 0].plot(self.time_stamps, self.X_state[:,
                           7].flatten(), marker="o", markersize=4)
            axs[0, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                           3].flatten(), marker="o", markersize=4)
            axs[0, 0].legend(['$v_x$ (UAV)', '$v_x$ (AMR)'])
            # axs[0, 0].set_title('X-velocity of uav and autonomous mobile robot(AMR)')
            axs[0, 0].set_ylabel('$v_x$ [m/s]')
            axs[0, 1].plot(self.time_stamps, self.X_state[:, 7].flatten(
            ) - self.mobile_robot_states[:, 3].flatten(), marker="o", markersize=4)
            # axs[0, 1].set_title(
            #     'X-velocity error between uav and autonomous mobile robot(AMR)')
            axs[0, 1].set_ylabel('$v_x$ (UAV)- $v_x$ (AMR) [m/s]')

            axs[1, 0].plot(self.time_stamps, self.X_state[:,
                           8].flatten(), marker="o", markersize=4)
            axs[1, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                           4].flatten(), marker="o", markersize=4)
            axs[1, 0].legend(['$v_y$ (UAV)', '$v_y$ (AMR)'])
            # axs[1, 0].set_title('Y-velocity of uav and autonomous mobile robot(AMR)')
            axs[1, 0].set_ylabel('$v_y$ [m/s]')
            axs[1, 0].set_xlabel('time [s]')
            axs[1, 1].plot(self.time_stamps, self.X_state[:, 8].flatten() - self.mobile_robot_states[:, 4].flatten(),
                           marker="o", markersize=4)
            # axs[1, 1].set_title(
            #     'Y-velocity error between uav and autonomous mobile robot(AMR)')
            axs[1, 1].set_ylabel('$v_y$ (UAV)- $v_y$ (AMR) [m/s]')
            axs[1, 1].set_xlabel('time [s]')
        else:
            axs[0, 0].plot(self.time_stamps, self.X_state[:,
                           6].flatten(), marker="o", markersize=4)
            axs[0, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                           3].flatten(), marker="o", markersize=4)
            axs[0, 0].legend(['$v_x$ (UAV)', '$v_x$ (AMR)'])
            # axs[0, 0].set_title('X-velocity of uav and autonomous mobile robot(AMR)')
            axs[0, 0].set_ylabel('$v_x$ [m/s]')
            axs[0, 1].plot(self.time_stamps, self.X_state[:, 6].flatten(
            ) - self.mobile_robot_states[:, 3].flatten(), marker="o", markersize=4)
            # axs[0, 1].set_title(
            #     'X-velocity error between uav and autonomous mobile robot(AMR)')
            axs[0, 1].set_ylabel('$v_x$ (UAV)- $v_x$ (AMR) [m/s]')

            axs[1, 0].plot(self.time_stamps, self.X_state[:,
                           7].flatten(), marker="o", markersize=4)
            axs[1, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                           4].flatten(), marker="o", markersize=4)
            axs[1, 0].legend(['$v_y$ (UAV)', '$v_y$ (AMR)'])
            # axs[1, 0].set_title('Y-velocity of uav and autonomous mobile robot(AMR)')
            axs[1, 0].set_ylabel('$v_y$ [m/s]')
            axs[1, 0].set_xlabel('time [s]')
            axs[1, 1].plot(self.time_stamps, self.X_state[:, 7].flatten() - self.mobile_robot_states[:, 4].flatten(),
                           marker="o", markersize=4)
            # axs[1, 1].set_title(
            #     'Y-velocity error between uav and autonomous mobile robot(AMR)')
            axs[1, 1].set_ylabel('$v_y$ (UAV)- $v_y$ (AMR) [m/s]')
            axs[1, 1].set_xlabel('time [s]')
        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                ymin, ymax = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(
                    patches.Rectangle(
                        (self.start_time, ymin),
                        self.time_interval,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='mistyrose',
                        fill=True
                    ))
                for _ in self.overlap_time:
                    axs[i, j].add_patch(
                        patches.Rectangle(
                            (_[0], ymin),
                            self.dt,  # width
                            rec_height,  # height
                            edgecolor='None',
                            facecolor='cyan',
                            fill=True
                        ))
        if savePNG:
            plt.savefig('uav_robot_comparision.png', format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('uav_robot_comparision.svg', format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_manipulator_robot_speed_comparison(self, savePNG=False, saveSVG=False, dpi=400):
        fig, axs = plt.subplots(2, 2, sharex='col',
                                figsize=(19 / 2.54, 10 / 2.54))  # use /2.54 to convert from cm to inches
        fig.suptitle("manipulator and robot speed comparision")
        axs[0, 0].plot(self.time_stamps, self.manipulator_states[:,
                       3].flatten(), marker="o", markersize=4)
        axs[0, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                       3].flatten(), marker="o", markersize=4)
        axs[0, 0].legend(['$v_x$ (manip.)', '$v_x$ (AMR)'])
        # axs[0, 0].set_title(
        #     'X-velocity of manipulator and mobile ground robot(AMR)')
        axs[0, 0].set_ylabel('$v_x$ [m/s]')
        axs[0, 1].plot(self.time_stamps, self.manipulator_states[:, 3].flatten() - self.mobile_robot_states[:, 3].flatten(),
                       marker="o", markersize=4)
        # axs[0, 1].set_title(
        # 'X-velocity error between manipulator and mobile ground robot(AMR)')
        axs[0, 1].set_ylabel('$\delta v_x$ ([m/s]')
        axs[1, 0].plot(self.time_stamps, self.manipulator_states[:,
                       4].flatten(), marker="o", markersize=4)
        axs[1, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                       4].flatten(), marker="o", markersize=4)
        axs[1, 0].legend(['$v_y$ (manip.)', '$v_y$(AMR)'])
        # axs[1, 0].set_title(
        #     'Y-velocity of manipulator and autonomous mobile robot(AMR)')
        axs[1, 0].set_ylabel('$v_y$ [m/s]')
        axs[1, 0].set_xlabel('time [s]')
        axs[1, 1].plot(self.time_stamps, self.manipulator_states[:, 4].flatten() - self.mobile_robot_states[:, 4].flatten(),
                       marker="o", markersize=4)
        # axs[1, 1].set_title(
        #     'Y-velocity error between manipulator and autonomous mobile robot(AMR)')
        axs[1, 1].set_ylabel('$\delta v_y$ [m/s]')
        axs[1, 1].set_xlabel('time [s]')
        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                ymin, ymax = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(
                    patches.Rectangle(
                        (self.start_time, ymin),
                        self.time_interval,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='mistyrose',
                        fill=True
                    ))
                for _ in self.overlap_time:
                    axs[i, j].add_patch(
                        patches.Rectangle(
                            (_[0], ymin),
                            self.dt,  # width
                            rec_height,  # height
                            edgecolor='None',
                            facecolor='cyan',
                            fill=True
                        ))
        if savePNG:
            plt.savefig('manipulator_robot_speed_comparision.png',
                        format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('manipulator_robot_speed_comparision.svg',
                        format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_manipulator_robot_position_comparison(self, savePNG=False, saveSVG=False, dpi=400):
        fig, axs = plt.subplots(3, 2, sharex='col',
                                figsize=(19 / 2.54, 10 / 2.54))  # use /2.54 to convert from cm to inches
        fig.suptitle("manipulator and robot position comparision")
        axs[0, 0].plot(self.time_stamps, self.manipulator_states[:,
                       0].flatten(), marker="o", markersize=4)
        axs[0, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                       0].flatten(), marker="^", markersize=4)
        axs[0, 0].legend(['$x$(manip.)', '$x$(AMR)'])
        # axs[0, 0].set_title(
        #     'X-position of manipulator and autonomous mobile robot(AMR)')
        axs[0, 0].set_ylabel('$x$ [m]')
        axs[0, 1].plot(self.time_stamps, self.manipulator_states[:, 0].flatten() - self.mobile_robot_states[:, 0].flatten(),
                       marker="o", markersize=4)
        # axs[0, 1].set_title(
        #     'X-position error between manipulator and autonomous mobile robot(AMR)')
        axs[0, 1].set_ylabel('$\delta x$ (manip.)-(AMR) [m]')

        axs[1, 0].plot(self.time_stamps, self.manipulator_states[:,
                       1].flatten(), marker="o", markersize=4)
        axs[1, 0].plot(self.time_stamps, self.mobile_robot_states[:,
                       1].flatten(), marker="^", markersize=4)
        axs[1, 0].legend(['$y$(manip.)', '$y$(AMR)'])
        # axs[1, 0].set_title(
        #     'Y-position of manipulator and autonomous mobile robot(AMR)')
        axs[1, 0].set_ylabel('$y$ [m]')
        axs[1, 1].plot(self.time_stamps, self.manipulator_states[:, 1].flatten() - self.mobile_robot_states[:, 1].flatten(),
                       marker="o", markersize=4)
        # axs[1, 1].set_title(
        #     'Y-position error between manipulator and autonomous mobile robot(AMR)')
        axs[1, 1].set_ylabel('$\delta y$ (manip.)-(AMR) [m]')

        axs[2, 0].plot(self.time_stamps, self.manipulator_states[:,
                       2].flatten(), marker="o", markersize=4)
        axs[2, 0].plot(self.time_stamps, self.mobile_robot_states[:, 2],
                       marker="^", markersize=4)
        axs[2, 0].legend(['$z$(manip.)', '$z$(AMR)'])
        # axs[2, 0].set_title(
        #     'Z-position of manipulator and autonomous mobile robot(AMR)')
        axs[2, 0].set_ylabel('$z$ [m]')
        axs[2, 0].set_xlabel('time [s]')
        axs[2, 1].plot(self.time_stamps, self.manipulator_states[:, 2].flatten(
        ) - self.mobile_robot_states[:, 2].flatten(), marker="o", markersize=4)
        # axs[2, 1].set_title(
        #     'Z-position error between manipulator and autonomous mobile robot(AMR)')
        axs[2, 1].set_ylabel('$\delta z$ (manip.)-(AMR) [m]')
        axs[2, 1].set_xlabel('time [s]')

        for i in range(np.shape(axs)[0]):
            for j in range(np.shape(axs)[1]):
                axs[i, j].grid(True)
                ymin, ymax = axs[i, j].get_ylim()
                rec_height = np.absolute(ymin) + np.absolute(ymax)
                axs[i, j].add_patch(
                    patches.Rectangle(
                        (self.start_time, ymin),
                        self.time_interval,  # width
                        rec_height,  # height
                        edgecolor='None',
                        facecolor='mistyrose',
                        fill=True
                    ))
                for _ in self.overlap_time:
                    axs[i, j].add_patch(
                        patches.Rectangle(
                            (_[0], ymin),
                            self.dt,  # width
                            rec_height,  # height
                            edgecolor='None',
                            facecolor='cyan',
                            fill=True
                        ))
        if savePNG:
            plt.savefig('manipulator_robot_position_comparision.png',
                        format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('manipulator_robot_position_comparision.svg',
                        format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_distance_manipulator_and_ground_robot(self, savePNG=False, saveSVG=False, dpi=400):
        _, axs = plt.subplots()
        axs.plot(self.time_stamps, self.dis_, marker="o", markersize=4)
        axs.set_title(
            'distance between manipulator and autonomous mobile robot(AMR)')
        axs.set_ylabel('distance [m]')
        axs.set_xlabel('time [s]')
        axs.grid(True)
        ymin, ymax = axs.get_ylim()
        rec_height = np.absolute(ymin) + np.absolute(ymax)
        axs.add_patch(
            patches.Rectangle(
                (self.start_time, ymin),
                self.time_interval,  # width
                rec_height,  # height
                edgecolor='None',
                facecolor='mistyrose',
                fill=True
            ))
        for _ in self.overlap_time:
            axs.add_patch(
                patches.Rectangle(
                    (_[0], ymin),
                    self.dt,  # width
                    rec_height,  # height
                    edgecolor='None',
                    facecolor='cyan',
                    fill=True
                ))
        if savePNG:
            plt.savefig('distance_manipulator_and_ground_robot.png',
                        format='png', dpi=dpi)
        if saveSVG:
            plt.savefig('distance_manipulator_and_ground_robot.svg',
                        format='svg', dpi=dpi)
        if not savePNG and not saveSVG:
            plt.show()

    def plot_all(self, savePNG=False, saveSVG=False, dpi=400):
        self.plot_uav_position_velocity(savePNG, saveSVG, dpi)
        self.plot_uav_attitude(savePNG, saveSVG, dpi)
        self.plot_manipulator_state(savePNG, saveSVG, dpi)
        self.plot_target_robot_state(savePNG, saveSVG, dpi)
        self.plot_uav_robot_comparison(savePNG, saveSVG, dpi)
        self.plot_manipulator_robot_position_comparison(savePNG, saveSVG, dpi)
        self.plot_manipulator_robot_speed_comparison(savePNG, saveSVG, dpi)
        self.plot_distance_manipulator_and_ground_robot(savePNG, saveSVG, dpi)


#########
# handle the data (csv)
#########


def import_uav_states(file_name, with_moving_manipulator=True):

    current_path = os.path.split(os.path.realpath(__file__))[0]
    df2 = pd.read_csv(r'{}/{}'.format(current_path, file_name)).to_numpy()

    if with_moving_manipulator:
        ref_x = df2[:, 0].reshape(1, -1)
        ref_y = df2[:, 1].reshape(1, -1)
        ref_z = df2[:, 2].reshape(1, -1)
        ref_phi = df2[:, 3].reshape(1, -1)
        ref_theta = df2[:, 4].reshape(1, -1)
        ref_psi = df2[:, 5].reshape(1, -1)
        ref_alpha = df2[:, 6].reshape(1, -1)

        ref_dx = df2[:, 7].reshape(1, -1)
        ref_dy = df2[:, 8].reshape(1, -1)
        ref_dz = df2[:, 9].reshape(1, -1)
        ref_dphi = df2[:, 10].reshape(1, -1)
        ref_dtheta = df2[:, 11].reshape(1, -1)
        ref_dpsi = df2[:, 12].reshape(1, -1)
        ref_dalpha = df2[:, 13].reshape(1, -1)

        ref_U1 = df2[:, 14]
        ref_U1 = ref_U1[~np.isnan(ref_U1)].reshape(1, -1)
        ref_U2 = df2[:, 15]
        ref_U2 = ref_U2[~np.isnan(ref_U2)].reshape(1, -1)
        ref_U3 = df2[:, 16]
        ref_U3 = ref_U3[~np.isnan(ref_U3)].reshape(1, -1)
        ref_U4 = df2[:, 17]
        ref_U4 = ref_U4[~np.isnan(ref_U4)].reshape(1, -1)
        ref_U5 = df2[:, 18]
        ref_U5 = ref_U5[~np.isnan(ref_U5)].reshape(1, -1)

        return ref_x, ref_y, ref_z, ref_phi, ref_theta, ref_psi, ref_alpha,\
            ref_dx, ref_dy, ref_dz, ref_dphi, ref_dtheta, ref_dpsi, ref_dalpha,\
            ref_U1, ref_U2, ref_U3, ref_U4, ref_U5

    else:
        ref_x = df2[:, 0].reshape(1, -1)
        ref_y = df2[:, 1].reshape(1, -1)
        ref_z = df2[:, 2].reshape(1, -1)
        ref_phi = df2[:, 3].reshape(1, -1)
        ref_theta = df2[:, 4].reshape(1, -1)
        ref_psi = df2[:, 5].reshape(1, -1)
        ref_dx = df2[:, 6].reshape(1, -1)
        ref_dy = df2[:, 7].reshape(1, -1)
        ref_dz = df2[:, 8].reshape(1, -1)
        ref_dphi = df2[:, 9].reshape(1, -1)
        ref_dtheta = df2[:, 10].reshape(1, -1)
        ref_dpsi = df2[:, 11].reshape(1, -1)
        ref_U1 = df2[:, 12]
        ref_U1 = ref_U1[~np.isnan(ref_U1)].reshape(1, -1)
        ref_U2 = df2[:, 13]
        ref_U2 = ref_U2[~np.isnan(ref_U2)].reshape(1, -1)
        ref_U3 = df2[:, 14]
        ref_U3 = ref_U3[~np.isnan(ref_U3)].reshape(1, -1)
        ref_U4 = df2[:, 15]
        ref_U4 = ref_U4[~np.isnan(ref_U4)].reshape(1, -1)
        return [ref_x, ref_y, ref_z, ref_phi, ref_theta, ref_psi, ref_dx, ref_dy, ref_dz, ref_dphi, ref_dtheta, ref_dpsi, ref_U1, ref_U2, ref_U3, ref_U4]


def import_assist_data(file_name):
    current_path = os.path.split(os.path.realpath(__file__))[0]
    df2 = pd.read_csv(r'{}/{}'.format(current_path, file_name)).to_numpy()
    # calculate how many lambda_param and mu_param
    val_ = int((np.shape(df2)[1] - 31) / 2)

    x_ground = df2[:, 0].reshape(1, -1)
    y_ground = df2[:, 1].reshape(1, -1)
    vx_ground = df2[:, 2].reshape(1, -1)
    vy_ground = df2[:, 3].reshape(1, -1)

    x_manip = df2[:, 4].reshape(1, -1)
    y_manip = df2[:, 5].reshape(1, -1)
    z_manip = df2[:, 6].reshape(1, -1)
    vx_manip = df2[:, 7].reshape(1, -1)
    vy_manip = df2[:, 8].reshape(1, -1)
    vz_manip = df2[:, 9].reshape(1, -1)

    # if val_ > 1:
    #     lambda_param = df2[:, 10:10 + val_].T
    #     mu_param = df2[:, 10 + val_: 10 + 2 * val_].T
    # else:
    lambda_param = df2[:, 10].reshape(1, -1)
    mu_param = df2[:, 11].reshape(1, -1)

    epsilon_param = df2[:, 10 + 2 * val_].reshape(1, -1)
    d_param = df2[:, 13].reshape(1, -1)
    l_param = df2[:, 14].reshape(1, -1)
    engage_index = df2[:, 15]
    engage_index = engage_index[~np.isnan(
        engage_index)].reshape(1, -1)
    Tn = df2[:, 16]
    Tn = Tn[~np.isnan(Tn)]
    N = df2[:, 17]
    N = N[~np.isnan(N)]

    pic = df2[:, 18]
    pickup_point = pic[~np.isnan(pic)]

    waypoints = df2[:, 19:]
    waypoints = waypoints[~np.isnan(waypoints)].reshape(-1, 12)

    return x_ground, y_ground, vx_ground, vy_ground, \
        x_manip, y_manip, z_manip, vx_manip, vy_manip, vz_manip, \
        lambda_param, mu_param, epsilon_param, d_param, l_param, \
        engage_index, Tn, N, pickup_point, waypoints


class handle_dmoc_results(object):
    def __init__(self, ):
        pass

    def get_dmoc_x_u(self, opt_x, opt_u, Tn, N):
        if opt_x.shape[1] == 12:
            print("no manipulator angle specified")
            self.with_moving_manipulator = False
            self.opt_x = opt_x
            self.opt_u = opt_u
            self.Tn = Tn
            self.N = N
        elif opt_x.shape[1] == 14:
            self.with_moving_manipulator = True
            self.opt_x = opt_x
            self.opt_u = opt_u
            self.Tn = Tn
            self.N = N

        else:
            print("the shape of x is not correct.")

    def get_manipulator_state(self, manipulator_state):
        self.manipulator_states = manipulator_state

    def get_uav_desired_path_waypoints(self, waypoints):
        self.uav_desired_path_waypoints = waypoints

    def get_target_trajectory(self, target_trajectory, assist_point):
        self.mobile_robot_states = target_trajectory
        self.assist_point = assist_point

    def get_engaged_list(self, engage_index, l_param):
        self.engage_index = engage_index
        self.relax_param = l_param

    def save_loaded_data(self, file_name):
        with open(file_name, 'wb') as f:
            np.save(f, self.Tn)
            np.save(f, self.N)
            np.save(f, self.opt_x)
            np.save(f, self.opt_u)
            np.save(f, self.manipulator_states)
            np.save(f, self.with_moving_manipulator)
            np.save(f, self.uav_desired_path_waypoints)
            np.save(f, self.mobile_robot_states)
            np.save(f, self.assist_point)
            np.save(f, self.engage_index)
            np.save(f, self.relax_param)

    def load_saved_data(self, file_name):
        with open(file_name, 'rb') as f:
            self.Tn = np.load(f, allow_pickle=True)
            self.N = np.load(f, allow_pickle=True)
            self.opt_x = np.load(f, allow_pickle=True)
            self.opt_u = np.load(f, allow_pickle=True)
            self.manipulator_states = np.load(f, allow_pickle=True)
            self.with_moving_manipulator = np.load(f, allow_pickle=True)
            self.uav_desired_path_waypoints = np.load(f, allow_pickle=True)
            self.mobile_robot_states = np.load(f, allow_pickle=True)
            self.assist_point = np.load(f, allow_pickle=True)
            self.engage_index = np.load(f, allow_pickle=True)
            self.relax_param = np.load(f, allow_pickle=True)
            print("reading data accomplished")
