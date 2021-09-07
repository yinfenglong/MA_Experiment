#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-06-01 11:47:52
LastEditors: Wei Luo
LastEditTime: 2021-07-14 10:09:07
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
from nmpc_acados_uav import export_uav_model


acados_source_path = os.environ['ACADOS_SOURCE_DIR']
sys.path.insert(0, acados_source_path)
model, constraints = export_uav_model()

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
# ocp.parameter_values = np.array([2.477, 0.47, 2.477, 0.47])
ocp.parameter_values = np.array([0.930, 0.137, 0.968, 0.138])
# ocp.parameter_values = np.array([0.85, 0.2, 0.86, 0.3])
# cost function
# Q_m = np.diag([20,  # x
#                20,  # y
#                20,  # z
#                0.3,  # vx
#                0.3,  # vy
#                0.3,  # vz
#                0.05,  # roll
#                0.05,  # pitch
#                0.05  # yaw
#                ])
# P_m = np.diag([
#     10,  # x
#     10,  # y
#     10,  # z
#     0.05,  # vx
#     0.05,  # vy
#     0.05  # vz
# ])

# R_m = np.diag([3.0, 3.0, 1.0])
Q_m = np.diag([80.0, 80.0, 120.0, 20.0, 20.0,
               30.0, 10.0, 10.0, 0.0])  # position, velocity, roll, pitch, (not yaw)

P_m = np.diag([86.21, 86.21, 120.95,
               6.94, 6.94, 11.04])  # only p and v
P_m[0, 3] = 6.45
P_m[3, 0] = 6.45
P_m[1, 4] = 6.45
P_m[4, 1] = 6.45
P_m[2, 5] = 10.95
P_m[5, 2] = 10.95
R_m = np.diag([50.0, 60.0, 1.0])

ocp.cost.cost_type = 'LINEAR_LS'
ocp.cost.cost_type_e = 'LINEAR_LS'
ocp.cost.W = scipy.linalg.block_diag(Q_m, R_m)
ocp.cost.W_e = P_m

ocp.cost.Vx = np.zeros((ny, nx))
ocp.cost.Vx[:nx, :nx] = np.eye(nx)
ocp.cost.Vu = np.zeros((ny, nu))
ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
ocp.cost.Vx_e = np.zeros((nx-3, nx))  # 3: euler angles
ocp.cost.Vx_e[:nx-3, :nx-3] = np.eye(nx-3)

# initial reference trajectory_ref
g = 9.8066
x_ref = np.zeros(nx)
x_ref_e = np.zeros(nx-3)
u_ref = np.zeros(nu)
u_ref[-1] = g
ocp.cost.yref = np.concatenate((x_ref, u_ref))
ocp.cost.yref_e = x_ref_e

# Set constraints
ocp.constraints.x0 = x_ref
ocp.constraints.lbu = np.array(
    [constraints.roll_min, constraints.pitch_min, constraints.thrust_min])
ocp.constraints.ubu = np.array(
    [constraints.roll_max, constraints.pitch_max, constraints.thrust_max])
ocp.constraints.idxbu = np.array([0, 1, 2])

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
export_path = join(dirname(dirname(abspath(__file__))), "solver_acados/uav")

if exists(export_path):
    shutil.rmtree(export_path)

shutil.move(generated_path, export_path)
shutil.copy(json_file_name, export_path)
os.remove(json_file_name)

print("files are generated in {0}. Please don't forget to 'catkin build' the ROS workspace once again".format(
    export_path))
