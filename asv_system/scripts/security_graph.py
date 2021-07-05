#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

y = []
y2 = []
f = open('/home/soubi/Documents/SEAOWL/nonor_ws/src/ros_asv_system/asv_system/output/210701123400.txt','r')
g = open('/home/soubi/Documents/SEAOWL/nonor_ws/src/ros_asv_system/asv_system/output/210701131840.txt','r')

for line in f :
    content = line.split()
    y.append(float(content[2]))
for line in g :
    content = line.split()
    y2.append(float(content[2]))

#plt.plot(y, c='b')
#plt.plot(y2, c='g')
plt.plot(y2[:len(y)], y, 'o', c='b')

plt.show()
f.close()
