import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

N = 1000
x = np.linspace(0,1,N)**2
y = x**2+(np.random.rand(N)-.5)/3
plt.plot(x,y)
z = savgol_filter(y, 101, 1)
plt.plot(x,z)
dz = np.gradient(z,x)

#plt.plot(x,dz)
#plt.plot(x,dy,'b')

#plt.plot(x,ddy,'b')

plt.show()