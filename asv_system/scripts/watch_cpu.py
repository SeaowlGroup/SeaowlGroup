#!/usr/bin/env python3

from psutil import cpu_percent, cpu_count
import time
import matplotlib.pyplot as plt

N = cpu_count()
result = []
for i in range(N):
    result.append([])

while (1):
    use = cpu_percent(percpu=True)
    for i in range(N):
        # print(result[i])
        # print(use[i])
        result[i].append(use[i])
    try:
        time.sleep(0.5)
    except KeyboardInterrupt:
        break

for i in range(N):
    plt.plot(result[i])
plt.show()
