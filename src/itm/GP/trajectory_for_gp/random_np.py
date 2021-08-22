#!/usr/bin/env python3
# coding=utf-8
import numpy as np

np.random.seed(50)
# a = 0.01 * np.random.randint( low=[0,0,40], high=[80,80,200], size=(50,3) )
x_min = -70
x_max = 70
y_min = -70
y_max = 70
z_min = 50
z_max = 170
a = 0.01 * np.random.randint( low=[x_min, y_min, z_min], high=[x_max, y_max, z_max], size=(1,3) )

random_matrix = np.array([[0,0,0.4]])
while (random_matrix.shape[0]<21):
    a = 0.01 * np.random.randint( low=[x_min, y_min, z_min], high=[x_max, y_max, z_max], size=(20,3) )
    for i in range(a.shape[0]):
        x_next, y_next, z_next = a[i,0], a[i,1], a[i,2]
        last_index = random_matrix.shape[0] - 1
        x, y, z = random_matrix[last_index,0], random_matrix[last_index,1], random_matrix[last_index,2]
        
        if abs(z-z_next)<0.3:
            # calculate distance square and distance should be in (0.1~0.5)
            distance = abs( (x-x_next)**2 + (y-y_next)**2 + (z-z_next)**2 )
            if distance > 0.01 and distance < 0.25:
                print("a",a)
                print(" insert a[i]", a[i])
                random_matrix = np.append( random_matrix, a[i].reshape(1,3), axis=0 )
                # np.append( (random_matrix,a[i].reshape(1,3)), axis=0 )
                if (random_matrix.shape[0]>=21):
                    print("random_matrix has enough values")
                    break
print("a:",a)
print("random_matrix:", random_matrix)
print("random_matrix shape:", random_matrix.shape)

