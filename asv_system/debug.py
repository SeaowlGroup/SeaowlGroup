import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter as sgf

data = np.loadtxt('/home/adrien/catkin_ws/src/seaowl/asv_system/debug.txt',skiprows=0)
t = data[:,0]
x = data[:,1]
#x = sgf(x,101,0)
y = data[:,2]
vx = data[:,3]
vy = data[:,4]
ax = data[:,5]
ay = data[:,6]

w = 11
d = 2
sx = sgf(x,w,d)
sy = sgf(y,w,d)
#vx = sgf(vx,101,0)
#vy = sgf(vy,101,0)

dx = np.gradient(x,t)
dvx = np.gradient(vx,t)
ddx = np.gradient(dx,t)

dsx = np.gradient(sx,t)
ddsx = np.gradient(dsx,t)

dy = np.gradient(y,t)
dvy = np.gradient(vy,t)
ddy = np.gradient(dy,t)

dsy = np.gradient(sy,t)
dvy = np.gradient(vy,t)
ddsy = np.gradient(dsy,t)

sdsx = sgf(dsx,w,d-1)
sdsy = sgf(dsy,w,d-1)
dsdsx = np.gradient(sdsx,t)
dsdsy = np.gradient(sdsy,t)

sdsdsx = sgf(dsdsx,w,d-2)
sdsdsy = sgf(dsdsy,w,d-2)

a = np.linalg.norm(np.array([sdsdsx,sdsdsy]),axis = 0)
#plt.plot(x,y)
#plt.gca().set_aspect('equal', adjustable='box')

#plt.plot(t[w//2+1:-(w//2+1)],x[w//2+1:-(w//2+1)])
#plt.plot(t[1:],vx[1:],'r')
#plt.plot(t,dx,'b')
#plt.plot(t[2:],ax[2:],'r')
#plt.plot(t[2:],dvx[2:],'b')
#plt.plot(t,ddx,'g')
#plt.plot(t,ddsx,'b')
#plt.plot(t,dsdsx,'r')
#plt.plot(t,sdsdsx,'k')

#plt.plot(t,a)
#plt.plot(t,y)
#plt.plot(t[1:],vy[1:],'r')
#plt.plot(t,dy,'b')
#plt.plot(t,ay,'r')
#plt.plot(t[2:],dvy[2:],'b')
#plt.plot(t,ddy,'r')
#plt.plot(t,ddy,'g')
#plt.plot(t,ddsy,'b')
#plt.plot(t,dsdsy,'r')
#plt.plot(t,sdsdsy,'k')


#plt.plot(t[w//2+2:-(w//2+2)],ddsy[w//2+2:-(w//2+2)],'k')
#plt.plot(t[w//2+2:-(w//2+2)],ddy[w//2+2:-(w//2+2)],'g')


plt.show()