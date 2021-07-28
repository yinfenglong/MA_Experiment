#!/usr/bin/env python
# coding=utf-8

import sys
sys.getdefaultencoding()
import pickle
# pickle.dump(model_name,open(,),protocol=2)
import numpy as np
np.set_printoptions(threshold=1000000000000000)
path = './drag__motor_noise__noisy__no_payload_7_0.pkl'
file = open(path,'rb')
# inf = pickle.load(file,encoding='iso-8859-1')       #读取pkl文件的内容
inf = pickle.load(file)       #读取pkl文件的内容
print(inf)
#fr.close()
inf=str(inf)
obj_path = '/test1.txt'
ft = open(obj_path, 'w')
ft.write(inf)

