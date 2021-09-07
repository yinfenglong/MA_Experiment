#!/usr/bin/env python
# coding=utf-8

import json
import time
import rospy
import threading
import numpy as np
import pandas as pd

def make_record_dict(state_dim):
    blank_recording_dict = {
            "state_in": np.zeros((0, state_dim)),
            "state_ref": np.zeros((0, state_dim)),
            "error": np.zeros((0, state_dim)),
            "input_in": np.zeros((0, 4)),
            "state_out": np.zeros((0, state_dim)),
            "state_pred": np.zeros((0, state_dim)),
            "timestamp": np.zeros((0, 1)),
            "dt": np.zeros((0, 1)),
    }
    return blank_recording_dict

rec_dict["dt"] = np.append(rec_dict["dt"], dt)
rec_dict["input_in"] = np.append(rec_dict["input_in"], w_opt[np.newaxis, :4], axis=0)
rec_dict["state_out"] = np.append(rec_dict["state_out"], state_final, 0

for key in self.rec_dict.keys():
    print(key, " ", self.rec_dict[key].shape)
    self.rec_dict[key] = jsonify(self.rec_dict[key])

df = pd.DataFrame(self.rec_dict)
df.to_csv(self.rec_file, index=True, mode='a', header=False)

def jsonify(array):
    if isinstance(array, np.ndarray):
        return array.tolist()
    if isinstance(array, list):
        return array
    return arra
