#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospkg


def plot_graph(i, j, serial, fig) :
    rospack = rospkg.RosPack()
    input = f"{rospack.get_path('asv_common')}/input/{serial}.txt"
    output = f"{rospack.get_path('asv_common')}/output/{serial}.txt"
    x = []
    y = []
    f1 = open(input,'r')
    f2 = open(output,'r')
    f1.readline()
    f2.readline()

    for line in f1:
        content = line.split()
        x.append(float(content[i]))
    for line in f2:
        content = line.split()
        y.append(float(content[j]))
    f1.close()
    f2.close()

    if len(x)==len(y):
        plt.plot(x,y, 'o')
    else:
        plt.plot(x,y[:len(x)], 'o')

    plt.show()

if __name__ == "__main__":
    plot_graph(0, 2, 210706155704)
