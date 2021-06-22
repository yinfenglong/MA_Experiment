#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-06-01 11:46:48
LastEditors: Wei Luo
LastEditTime: 2021-06-02 22:37:34
Note: Note
'''

import casadi as ca
import numpy as np
from acados_template import AcadosModel


def export_uav_model():
    g_ = 9.8066
    # control input
    roll_ref_ = ca.SX.sym('roll_ref')
    pitch_ref_ = ca.SX.sym('pitch_ref')
    thrust_ref_ = ca.SX.sym('thrust_ref')
    controls = ca.vcat([roll_ref_, pitch_ref_,
                        thrust_ref_])

    # model state
    x_ = ca.SX.sym('x')
    y_ = ca.SX.sym('y')
    z_ = ca.SX.sym('z')
    vx_ = ca.SX.sym('vx')
    vy_ = ca.SX.sym('vy')
    vz_ = ca.SX.sym('vz')
    roll_ = ca.SX.sym('roll')
    pitch_ = ca.SX.sym('pitch')
    yaw_ = ca.SX.sym('yaw')

    # states  [p, q, v]
    states = ca.vcat([x_, y_, z_, vx_, vy_, vz_, roll_, pitch_, yaw_])

    # roll_gain = 2.477
    # roll_tau = 0.477
    # pitch_gain = 2.477
    # pitch_tau = 0.477

    roll_gain = ca.SX.sym('roll_gain')
    roll_tau = ca.SX.sym('roll_tau')
    pitch_gain = ca.SX.sym('pitch_gain')
    pitch_tau = ca.SX.sym('pitch_tau')

    params = ca.vcat([roll_gain, roll_tau, pitch_gain, pitch_tau])

    rhs = [
        vx_,
        vy_,
        vz_,
        (ca.cos(roll_) * ca.cos(yaw_) * ca.sin(pitch_) +
         ca.sin(roll_) * ca.sin(yaw_)) * thrust_ref_,
        (ca.cos(roll_) * ca.sin(pitch_) * ca.sin(yaw_) -
         ca.cos(yaw_) * ca.sin(roll_)) * thrust_ref_,
        -g_ + ca.cos(pitch_) * ca.cos(roll_) * thrust_ref_,
        (roll_gain * roll_ref_ - roll_) / roll_tau,
        (pitch_gain * pitch_ref_ - pitch_) / pitch_tau,
        0.0
    ]

    f = ca.Function('f', [states, controls], [ca.vcat(rhs)])

    x_dot = ca.SX.sym('x_dot', len(rhs))
    f_impl = x_dot - f(states, controls)

    model = AcadosModel()
    model.f_expl_expr = f(states, controls)
    model.f_impl_expr = f_impl
    model.x = states
    model.xdot = x_dot
    model.u = controls
    model.p = params
    model.name = 'quadrotor'

    constraints = ca.types.SimpleNamespace()
    constraints.roll_min = np.deg2rad(-85)
    constraints.pitch_min = np.deg2rad(-85)
    constraints.roll_max = np.deg2rad(85)
    constraints.pitch_max = np.deg2rad(85)
    constraints.thrust_min = 0.5*g_
    constraints.thrust_max = 1.9*g_

    return model, constraints
