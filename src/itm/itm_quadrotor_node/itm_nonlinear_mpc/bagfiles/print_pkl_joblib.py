#!/usr/bin/env python
# coding=utf-8
'''
Date: 13.07.2021
Author: Yinfeng Long 
usage 
    python3 print_pkl_joblib.py filename.pkl 
'''

import sys
import joblib

# a_dict1 = joblib.load('./drag__motor_noise__noisy__no_payload_8_0.pkl')
a_dict1 = joblib.load(sys.argv[1])
print(a_dict1)
