import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LinearRegression

x = 0.514444*np.array([[1],[2],[5],[10],[15],[25]]) #in m/s
y1 = np.array([10,15,30,50,80,100]) #in m
y2 = np.array([0,10,15,30,50,80]) #in m
reg = LinearRegression().fit(x, y1)


plt.plot(x,y1,'+')
plt.plot(x,reg.predict(x))
print(reg.coef_,reg.intercept_) #t0 = 7.6s, d0 = 9.8m
plt.show()

#plt.plot(x,y2,'x')
