#!/usr/bin/env python
# coding=utf-8
from random_trajectory_generator import random_matrix_generator

matrix = random_matrix_generator(-70, 70, -70, 70, 30, 60, 200)
print(matrix)
print(matrix.shape)
