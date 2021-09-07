#!/usr/bin/env python
# coding=utf-8
'''
Date: 04.07.2021
Author: Yinfeng Long 
usage 
    python csvs_to_figure.py filename1.csv filename2.csv
TODO:
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

# data1 = pd.read_csv(sys.argv[1] , skiprows=lambda x: x>0 and (x-1)% 10 !=0 )
# data2 = pd.read_csv(sys.argv[2] , skiprows=lambda x: x>0 and (x-1)% 10 !=0 )
data1 = pd.read_csv(sys.argv[1] )
data2 = pd.read_csv(sys.argv[2] )

data1['rosbagTimestamp'] = data1['rosbagTimestamp'].apply(lambda x: pd.to_datetime(x))
data1.to_csv('output1.csv')
newdata1 = pd.read_csv('output1.csv')

data2['rosbagTimestamp'] = data2['rosbagTimestamp'].apply(lambda x: pd.to_datetime(x))
data2.to_csv('output2.csv')
newdata2 = pd.read_csv('output2.csv')

t2=pd.to_datetime(newdata2.iloc[1,1])
t1=pd.to_datetime(newdata1.iloc[1,1])
t_diff= (t2-t1).days
print("t2:", t2)
print("t1:", t1)
print((t2-t1).seconds)
print((t2-t1).microseconds)

if ( t_diff == 0):
    print(sys.argv[1], ' is eariler than ', sys.argv[2])
    time2 = pd.to_datetime(newdata2.iloc[0,1]) 
    data1_num = newdata1.shape[0]
    data2_num = newdata2.shape[0]
    for j in range(1, data1_num):
        time1 = pd.to_datetime(newdata1.iloc[j,1])
        time_diff_s = (time2-time1).seconds
        if (time_diff_s == 0):
            time_diff_ms = (time2-time1).microseconds
            if (time_diff_ms < 70000):
                print("data2: 0", "data1: ", j )

                fig = plt.figure(figsize=[9,5])
                time_2 = newdata2['rosbagTimestamp']
                time_1 = newdata1.loc[j:data1_num, ['rosbagTimestamp']]
                ax1 = fig.add_subplot(3,1,1)
		# time = newdata.iloc[:,1]
		# x = newdata.iloc[:,12]
		plt.xlabel('time')
                # ax1.set_xlim( newdata1.loc[j, ['rosbagTimestamp']], newdata1.loc[data1_num-1, ['rosbagTimestamp']] )
		# ax1.set_ylim(-1.0, 1.0)
                plt.ylabel('x')
		x_2 = newdata2['x']
		plt.plot(time_2,x_2,c='red')
                # x_1 = newdata1.loc[j:data1_num, ['x']]
                x_1 = newdata1.iloc[j:data1_num,10]
		plt.plot(time_1,x_1,c='orange')
                ax1.set( xlim = [ newdata1.loc[j, ['rosbagTimestamp']], newdata1.loc[data1_num-1, ['rosbagTimestamp']]] )

		ax2 = fig.add_subplot(3,1,2)
		# time = newdata.iloc[:,1] #This approach works, but a specific number of x'column need to be known
		# y = newdata.iloc[:,13]
		plt.xlabel('')
		plt.ylabel('y')
		y_2 = newdata2['y']
		plt.plot(time_2,y_2,c='green')
                y_1 = newdata1.loc[j:data1_num, ['y']]
		plt.plot(time_1,y_1,c='lime')

		ax3 = fig.add_subplot(3,1,3)
		# time = newdata.iloc[:,1]
		# z = newdata.iloc[:,14]
		plt.xlabel('')
		plt.ylabel('z')
		z_2 = newdata2['z']
		plt.plot(time_2,z_2,c='blue')
                z_1 = newdata1.loc[j:data1_num, ['z']]
		plt.plot(time_1,z_1,c='cyan')

		plt.show()
                break
            else:
                continue
        else:
            continue
            
elif ( t_diff == -1):
    print(sys.argv[2], " is eariler than ", sys.argv[1])

# for i in range(300):
    # for j in range(100):



''' ---{test time difference calculation
t2=pd.to_datetime(newdata.iloc[300,1])
t1=pd.to_datetime(newdata.iloc[0,1])
print ("t2=", t2)
print ("t1=", t1)
print((t2-t1).seconds)
print((t2-t1).microseconds)
}--- '''

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



