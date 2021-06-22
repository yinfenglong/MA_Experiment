#!/usr/bin/env python
# coding=utf-8
'''
Author: Wei Luo
Date: 2020-10-20 20:22:41
LastEditors: Wei Luo
LastEditTime: 2020-10-20 20:22:53
Note: Note
'''

import numpy
from distutils.core import setup, Extension

setup(name='acado_uav_rate', version='1.0',
     ext_modules=[Extension('acado_uav_rate', ['acado_uav.c',  \
          'acado_auxiliary_functions.c', \
          'acado_integrator.c',  \
          'acado_qpoases_interface.cpp', \
          'acado_solver.c', \
	  'qpoases/SRC/Bounds.cpp', \
	  'qpoases/SRC/Constraints.cpp', \
	  'qpoases/SRC/CyclingManager.cpp', \
	  'qpoases/SRC/Indexlist.cpp',  \
	  'qpoases/SRC/MessageHandling.cpp', \
	  'qpoases/SRC/QProblem.cpp', \
	  'qpoases/SRC/QProblemB.cpp', \
	  'qpoases/SRC/SubjectTo.cpp', \
	  'qpoases/SRC/Utils.cpp', \
	  'qpoases/SRC/EXTRAS/SolutionAnalysis.cpp', \
         ],
         include_dirs=['.', numpy.get_include(), 'qpoases', 'qpoases/INCLUDE', 'qpoases/SRC'])])
