import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter as sgf

def att(x,option) :
    #if x == 0.:
    #    return 1
    if option == 0:
        return np.minimum(1,1/x)
    if option == 1:
        return 1/(1+x)
    if option == 2:
        return np.maximum(1-x,0)
    if option == 3:
        return np.exp(-x)

data = np.loadtxt('/home/adrien/catkin_ws/src/seaowl/asv_system/debug.txt',skiprows=0)
t = data[:,0]
tcpa = t[-1]
t1 = 10

n_obst = (len(data[0,:])-3)//2

x = data[:,1]
y = data[:,2]
x_obst = data[:,3]
y_obst = data[:,4]

plt.plot(t,x)
plt.plot(t,x_obst)
plt.show()

w = 11
d = 2
sx = sgf(x,w,d)
sy = sgf(y,w,d)

dsx = np.gradient(sx,t)
dsy = np.gradient(sy,t)

sdsx = sgf(dsx,w,d-1)
sdsy = sgf(dsy,w,d-1)

dsdsx = np.gradient(sdsx,t)
dsdsy = np.gradient(sdsy,t)

sdsdsx = sgf(dsdsx,w,d-2)
sdsdsy = sgf(dsdsy,w,d-2)

v = np.linalg.norm(np.array([sdsx,sdsy]),axis = 0)
a = np.linalg.norm(np.array([sdsdsx,sdsdsy]),axis = 0)




weight = att((np.abs(tcpa -t))/t1,3)
weight = np.sqrt(weight/np.sum(weight))

#plt.plot(weight)
#plt.show()

#plt.plot(t,sx)
#plt.plot(t,sy)
#plt.show()

#plt.plot(t,sdsx)
#plt.plot(t,sdsy)
#plt.plot(t,v)
#plt.show()

#plt.plot(t,sdsdsx)
#plt.plot(t,sdsdsy)
#plt.plot(t,a)
#plt.show()