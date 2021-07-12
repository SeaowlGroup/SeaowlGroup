#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospkg
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def clear_frame(frame):
    for w in frame.winfo_children():
        w.destroy()

def plot_graph(i, j, serial, cut, lp, vo, g) :
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
    groups = []
    f1 = open(input,'r')
    f2 = open(output,'r')
    f1.readline()
    f2.readline()

    for line in f1:
        content = line.split()
        x.append(float(content[i]))
        groups.append(int(content[10]))
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

    for p in range(40):
        #if p!=13 and markers[p]!='o':
        if y[p] < cut and g[groups[p]-1].get()==1:
            if (lp==1 and markers[p]=='o') or (vo==1 and markers[p]=='v'):
                ax.plot(x[p], y[p], marker=markers[p], color=colors[p], label=labels[p])
    #ax.axhline(y[40], '--', color='lightgrey', label='witness 50')
    #ax.axhline(y[41], '--', color='black', label='witness 100')
    ax.axhline(y[40], linewidth=0.5, color='grey', label='witness 50')
    ax.axhline(y[41], linewidth=0.5, color='black', label='witness 100')

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

    return figure

if __name__ == "__main__":
    aux_frame = tk.Tk()

    graph_frame = tk.Frame(aux_frame, bg='white')
    graph_frame.grid(rowspan=2, column=1)

    frame1 = tk.Frame(aux_frame, bg='gainsboro', padx=5)
    frame2 = tk.Frame(aux_frame, bg='gainsboro')
    frame3 = tk.Frame(aux_frame, bg='gainsboro', padx=20)
    frame1.grid(row=0, column=0)
    frame2.grid(row=1, column=0)
    frame3.grid(row=0, column=2)
    tk.Label(frame1, text='X :', bg='gainsboro').pack()
    tk.Label(frame2, text='Y :', bg='gainsboro').pack()
    tk.Label(frame3, text='Parameters :', bg='gainsboro').pack()

    x = tk.IntVar()
    y = tk.IntVar()
    x.set(0)
    y.set(1)
    cutoff = tk.DoubleVar()
    cutoff.set(1000)
    lp = tk.IntVar()
    lp.set(1)
    vo = tk.IntVar()
    vo.set(1)
    groups = []

    def update_plot():
        clear_frame(graph_frame)
        fig = plot_graph(x.get(), y.get(), 'survivor2', cutoff.get(), lp.get(), vo.get(), groups)
        chart_type = FigureCanvasTkAgg(fig, graph_frame)
        chart_type.get_tk_widget().pack()
        cutoff.set(1000)

    x_list = [["Opus",0], ["Obstacle Heading",4], ["dCPA",6], ["Detection Distance",9]]
    y_list = [["Time",1], ["Natural Collision Indic.",3], ["Logarithmic Collision Indic.",2],
              ["Offset Collision Indic.",4], ["Anticipation Inv Indic.",5], ["Anticipation Off Indic.",6],
              ["Anticipation Lin Indic.",7], ["Anticipation Exp Indic.",8]]

    for w in x_list:
        tk.Radiobutton(frame1, variable=x, text=w[0], value=w[1], highlightthickness=0, command=update_plot).pack(fill='both')
    for w in y_list:
        tk.Radiobutton(frame2, variable=y, text=w[0], value=w[1], highlightthickness=0, command=update_plot).pack(fill='both')


    frame3_1 = tk.Frame(frame3)
    frame3_1.pack()
    tk.Label(frame3_1, text='Cutoff:').pack(side='left')
    tk.Entry(frame3_1, textvariable=cutoff, width=5).pack()
    tk.Button(frame3, text='Set Cutoff', command=update_plot).pack()
    tk.Checkbutton(frame3, text="Display LP", variable=lp, command=update_plot).pack()
    tk.Checkbutton(frame3, text="Display VO", variable=vo, command=update_plot).pack()

    for g in range(5):
        groups.append(tk.IntVar())
        groups[g].set(1)
        tk.Checkbutton(frame3, text=f"Display Group {g+1}", variable=groups[g], command=update_plot).pack()


    update_plot()

    aux_frame.mainloop()
