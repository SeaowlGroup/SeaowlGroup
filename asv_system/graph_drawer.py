#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospkg
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def clear_frame(frame):
    for w in frame.winfo_children():
        w.destroy()

def plot_graph(i, j, serial) :
    rospack = rospkg.RosPack()
    input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
    output = f"{rospack.get_path('asv_system')}/output/{serial}.txt"

    x = []
    y = []
    xlab = ['OPUS', 'CLASS', 'U_D_ASV', 'LOC_PLAN', 'HEADING', 'U_D', 'DCPA', 'SIZE', 'PRIOR', 'D_DETEC']
    ylab = ['TIME', 'LOG_COL', 'NAT_COL', 'OFFSET_LOG', 'ANTICIPATION_INV', 'ANTICIPATION_OFF', 'ANTICIPATION_LIN', 'ANTICIPATION_EXP']
    labels = []
    colors = []
    markers = []
    f1 = open(input,'r')
    f2 = open(output,'r')
    f1.readline()
    f2.readline()

    for line in f1:
        content = line.split()
        x.append(float(content[i]))
        if content[3] == "True":
            labels.append('Velocity Obstacles')
            markers.append('v')
        else:
            labels.append('No LP')
            markers.append('o')

        if content[1] == 'CF':
            colors.append('blue')
            labels[-1]+="/CF"
        elif content[1] == 'CL':
            colors.append('orange')
            labels[-1]+="/CL"
        elif content[1] == 'WITNESS':
            colors.append('purple')
            labels[-1]+="/Witness"
        else:
            colors.append('grey')


    for line in f2:
        content = line.split()
        y.append(float(content[j]))
    f1.close()
    f2.close()

    figure = plt.Figure(figsize=(8,8), dpi=100)

    ax = figure.add_subplot(111)
    ax.set_title("Results of the Simulation")
    ax.set_xlabel(xlab[i])
    ax.set_ylabel(ylab[j-1])

    n = min(len(x), len(y))

    for p in range(n):
        #if p!=13 and markers[p]!='o':
        if p!=13:
            ax.plot(x[p], y[p], marker=markers[p], color=colors[p], label=labels[p])

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

    return figure

if __name__ == "__main__":
    aux_frame = tk.Tk()

    graph_frame = tk.Frame(aux_frame, bg='white')
    graph_frame.pack()


    frame1 = tk.Frame(aux_frame, bg='gainsboro')
    frame2 = tk.Frame(aux_frame, bg='gainsboro')
    frame1.pack(side='left', expand='yes')
    frame2.pack(side='left', expand='yes')
    tk.Label(frame1, text='X :', bg='gainsboro').pack()
    tk.Label(frame2, text='Y :', bg='gainsboro').pack()

    x = tk.IntVar()
    y = tk.IntVar()
    x_list = [["Opus",0], ["Obstacle Heading",4], ["dCPA",6], ["Detection Distance",9]]
    y_list = [["Time",1], ["Natural Collision Indic.",3], ["Logarithmic Collision Indic.",2],
              ["Offset Collision Indic.",4], ["Anticipation Inv Indic.",5], ["Anticipation Off Indic.",6],
              ["Anticipation Lin Indic.",7], ["Anticipation Exp Indic.",8]]

    x.set(0)
    y.set(1)

    def update_plot():
        clear_frame(graph_frame)
        fig = plot_graph(x.get(), y.get(), 'survivor')
        chart_type = FigureCanvasTkAgg(fig, graph_frame)
        chart_type.get_tk_widget().pack()

    for w in x_list:
        tk.Radiobutton(frame1, variable=x, text=w[0], value=w[1], highlightthickness=0, command=update_plot).pack(fill='both')
    for w in y_list:
        tk.Radiobutton(frame2, variable=y, text=w[0], value=w[1], highlightthickness=0, command=update_plot).pack(fill='both')

    aux_frame.mainloop()
