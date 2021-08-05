#!/usr/bin/env python3

from psutil import cpu_percent, cpu_count, virtual_memory
import time
import matplotlib.pyplot as plt

N = cpu_count()
result = []
for i in range(N):
    result.append([])

mem_tot = virtual_memory().total
mem_min = mem_tot
mem_max = 0

while (1):
    use = cpu_percent(percpu=True)
    for i in range(N):
        # print(result[i])
        # print(use[i])
        result[i].append(use[i])

    mem = virtual_memory()
    mem_min = min(mem_min, mem_tot-mem.available)
    mem_max = max(mem_max, mem_tot-mem.available)

    try:
        time.sleep(0.5)
    except KeyboardInterrupt:
        break

print(f"\nMinimum memory use : {mem_min/1024/1024/1024} Go / {mem_tot/1024/1024/1024} Go")
print(f"Maximum memory use : {mem_max/1024/1024/1024} Go / {mem_tot/1024/1024/1024} Go")
print(f"Difference : {(mem_max-mem_min)/1024/1024/1024} Go")

for i in range(N):
    plt.plot(result[i])
plt.show()
