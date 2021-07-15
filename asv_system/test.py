import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

N = 1000
M = 2
x = np.linspace(0,1,N)**2
y = x**2+(np.random.rand(N)-.5)/3
#plt.plot(x,y)
z = savgol_filter(y, 101, 1)
#plt.plot(x,z)
dz = np.gradient(z,x)

def d(x):
    N = len(x)
    y = np.zeros(N)
    for k in range(N):
        if k == 0:
            y[k] = (-x[k+2]+4*x[k+1]-3*x[k])/2*N
        elif k == N-1:
            y[k] = -(-x[k-2]+4*x[k-1]-3*x[k])/2*N
        else:
            y[k] = (x[k+1]-x[k-1])/2*N
    return y  

t = np.linspace(0,1,N)
x = np.exp(t)
y = np.zeros((M,N))
z = np.zeros((M,N))

y[0,:] = x
z[0,:] = x

plt.plot(t,y[0,:])
for k in range(1,M):
    y[k,:] = d(y[k-1,:])
    #plt.plot(t[M:-M],y[k,M:-M])
    z[k,:] = np.gradient(z[k-1,:],t)
    plt.plot(t[M:-M],z[k,M:-M])




#plt.plot(t,y2)
#plt.plot(x,dy,'b')

#plt.plot(x,ddy,'b')

plt.show()