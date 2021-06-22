#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-06-01 11:47:52
LastEditors: Wei Luo
LastEditTime: 2021-06-04 00:05:31
Note: Note
'''

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import casadi as ca
import scipy.linalg
import numpy as np
from os.path import dirname, join, abspath, exists
import os
import shutil
import sys
from nmpc_acados_uav_q import export_uav_q_model


acados_source_path = os.environ['ACADOS_SOURCE_DIR']
sys.path.insert(0, acados_source_path)
model, constraints = export_uav_q_model()

# create OCP
ocp = AcadosOcp()
ocp.acados_include_path = acados_source_path + '/include'
ocp.acados_lib_path = acados_source_path + '/lib'
ocp.model = model
ocp.dims.N = 20
ocp.solver_options.tf = 2.


nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu

# additional parameters
n_params = model.p.size()[0] if isinstance(model.p, ca.SX) else 0
ocp.dims.np = n_params
ocp.parameter_values = np.zeros(n_params)

# cost function
Q_m = np.diag([20,  # x
               20,  # y
               20,  # z
               0.03,  # qw
               0.03,  # qx
               0.03,  # qy
               0.03,  # qz
               0.05,  # vx
               0.05,  # vy
               0.05  # vz
               ])
# Q_m = np.diag([10, 10, 10,
#                0.3, 0.3, 0.3, 0.3,
#                0.05, 0.05, 0.05,
#                ])  # position, q, v
P_m = np.diag([
    10,  # x
    10,  # y
    20,  # z
    0.05,  # vx
    0.05,  # vy
    0.05  # vz
])

# P_m = np.diag([10,  10, 10,
#                0.05, 0.05, 0.05])

R_m = np.diag([2.0, 2.0, 5.0, 0.6])

ocp.cost.cost_type = 'LINEAR_LS'
ocp.cost.cost_type_e = 'LINEAR_LS'
ocp.cost.W = scipy.linalg.block_diag(Q_m, R_m)
ocp.cost.W_e = P_m

ocp.cost.Vx = np.zeros((ny, nx))
ocp.cost.Vx[:nx, :nx] = np.eye(nx)
ocp.cost.Vu = np.zeros((ny, nu))
ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
ocp.cost.Vx_e = np.zeros((nx-4, nx))  # 4: quaternion
ocp.cost.Vx_e[:3, :3] = np.eye(3)
ocp.cost.Vx_e[-3:, -3:] = np.eye(3)

# initial reference trajectory_ref
g = 9.8066
x_ref = np.zeros(nx)
x_ref[3] = 1.0
x_ref_e = np.zeros(nx-4)
u_ref = np.zeros(nu)
u_ref[-1] = g
ocp.cost.yref = np.concatenate((x_ref, u_ref))
ocp.cost.yref_e = x_ref_e
ocp.constraints.x0 = x_ref

# Set constraints
ocp.constraints.lbu = np.array(
    [constraints.roll_rate_min, constraints.pitch_rate_min, constraints.yaw_rate_min, constraints.thrust_min])
ocp.constraints.ubu = np.array(
    [constraints.roll_rate_max, constraints.pitch_rate_max, constraints.yaw_rate_max, constraints.thrust_max])
ocp.constraints.idxbu = np.array([0, 1, 2, 3])

# solver options
ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
# explicit Runge-Kutta integrator
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.print_level = 0
ocp.solver_options.nlp_solver_type = 'SQP'  # 'SQP_RTI'
ocp.solver_options.nlp_solver_max_iter = 200

# compile acados ocp
json_file_name = model.name + '_acados_ocp.json'
if exists('c_generated_code'):
    shutil.rmtree('c_generated_code')
if exists(json_file_name):
    os.remove(json_file_name)
solver = AcadosOcpSolver(ocp, json_file=json_file_name)
integrator = AcadosSimSolver(
    ocp, json_file=json_file_name)

# move files to destination
generated_path = join(dirname(abspath(__file__)), "c_generated_code")
export_path = join(dirname(dirname(abspath(__file__))), "solver_acados/uav_q")

if exists(export_path):
    shutil.rmtree(export_path)

shutil.move(generated_path, export_path)
shutil.copy(json_file_name, export_path)
os.remove(json_file_name)

print("files are generated in {0}. Please don't forget to 'catkin build' the ROS workspace once again".format(
    export_path))
