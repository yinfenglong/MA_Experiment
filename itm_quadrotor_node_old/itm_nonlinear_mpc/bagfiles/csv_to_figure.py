#!/usr/bin/env python
# coding=utf-8
'''
Date: 23.06.2021
Author: Yinfeng Long 
usage 
    python csv_to_figure.py filename.csv
'''

import numpy as np
import matplotlib  
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import time
import sys
import pandas as pd
from datetime import datetime 
matplotlib.style.use('ggplot')

#data = pd.read_csv('_slash_robot_pose.csv' )
data = pd.read_csv(sys.argv[1] )

# time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(data[:,0]))
# data['date'] = map(lambda y: time.strftime('%Y-%m-%d',y), map(lambda x:time.localtime(x), data['rosbagTimestamp']))
data['rosbagTimestamp'] = data['rosbagTimestamp'].apply(lambda x: pd.to_datetime(x))
# data['rosbagTimestamp'] = data['rosbagTimestamp'].apply(lambda x: datetime.fromtimestamp(x))
# data.to_csv('output.csv')
data.to_csv('output.csv')

newdata = pd.read_csv('output.csv', skiprows=lambda x: x>0 and (x-1) % 50 !=0 )

fig = plt.figure(figsize=[9,5])
ax1 = fig.add_subplot(3,1,1)
# time = newdata.iloc[:,1]
# x = newdata.iloc[:,12]
# print(time)
time = newdata['rosbagTimestamp']
x = newdata['x']
plt.xlabel('')
plt.ylabel('x')
plt.plot(time,x,c='red')

ax2 = fig.add_subplot(3,1,2)
# time = newdata.iloc[:,1] #This approach works, but a specific number of x'column need to be known
# y = newdata.iloc[:,13]
time = newdata['rosbagTimestamp']
y = newdata['y']
plt.xlabel('')
plt.ylabel('y')
plt.plot(time,y,c='green')

ax3 = fig.add_subplot(3,1,3)
# time = newdata.iloc[:,1]
# z = newdata.iloc[:,14]
time = newdata['rosbagTimestamp']
z = newdata['z']
plt.xlabel('')
plt.ylabel('z')
plt.plot(time,z,c='blue')

"""
newdata = np.loadtxt('output.csv', delimiter=',', converters={1:mdates.strpdate2num('%Y-%m-%d%H:%M:%S')}, unpack=True)
# print(data)
# newdata = pd.read_csv('output.csv')
time = newdata[:,0]
x = newdata[:,11]
# y = data[:,12]
# z = data[:,13]

# plt.subplot(131)
plt.bar(time,x)
plt.xlabel('time')
plt.ylabel('x')

plt.subplot(132)
plt.bar(time,y)
plt.xlabel('time')
plt.ylabel('y')

plt.subplot(133)
plt.bar(time,z)
plt.xlabel('time')
plt.ylabel('z')
"""
plt.show()
