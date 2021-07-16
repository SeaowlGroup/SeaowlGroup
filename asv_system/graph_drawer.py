#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospkg
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle

def clear_frame(frame):
    for w in frame.winfo_children():
        w.destroy()

class Fig(object):

    def __init__(self, serial='survivor7', n_groups=5):
        rospack = rospkg.RosPack()
        self.input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
        self.output = f"{rospack.get_path('asv_system')}/output/{serial}.txt"

        self.i = 0
        self.j = 1
        self.cutoff = 1000000
        self.disp_lp = [True]*2
        self.disp_groups = [True]*n_groups
        self.annotate = False
        self.disp_witness = False
        self.validation_color = True

        self.xlab = ['OPUS', 'CLASS', 'U_D_ASV', 'LOC_PLAN', 'HEADING', 'U_D', 'DCPA', 'SIZE', 'PRIOR', 'D_DETEC']
        self.ylab = ['TIME', 'LOG_COL', 'NAT_COL', 'OFFSET_LOG', 'ANTICIPATION_INV', 'ANTICIPATION_OFF',
                     'ANTICIPATION_LIN', 'ANTICIPATION_EXP', 'DCPA', 'CROSSING_DIST']

        self.labels = []
        self.colors = []
        self.markers = []
        self.n_groups = n_groups
        self.groups = []
        f1 = open(self.input,'r')
        f1.readline()

        for line in f1:
            content = line.split()
            self.groups.append(int(content[10]))
            if content[3] == "True":
                self.labels.append('Velocity Obstacles')
                self.markers.append('v')
            else:
                self.labels.append('No LP')
                self.markers.append('o')

            if content[1] == 'CF':
                self.colors.append('blue')
                self.labels[-1]+="/CF"
            elif content[1] == 'CL':
                self.colors.append('orange')
                self.labels[-1]+="/CL"
            elif content[1] == 'WITNESS':
                self.colors.append('purple')
                self.labels[-1]+="/Witness"
            else:
                self.colors.append('grey')

        f1.close()


    def plot_graph(self) :
        x = []
        y = []

        f1 = open(self.input,'r')
        f2 = open(self.output,'r')
        f1.readline()
        f2.readline()

        for line in f1:
            content = line.split()
            #if (float(content[2]) > 1.5):
            x.append(float(content[self.i]))
        for line in f2:
            content = line.split()
            y.append(float(content[self.j]))

        f1.close()
        f2.close()

        figure = plt.Figure(figsize=(8,8), dpi=100)

        ax = figure.add_subplot(111)
        ax.set_title("Results of the Simulation")
        ax.set_xlabel(self.xlab[self.i])
        ax.set_ylabel(self.ylab[self.j-1])

        n = min(len(x), len(y))

        for p in range(n):
            #if p!=0 :
            if y[p] < self.cutoff and self.disp_groups[self.groups[p]-1]:
                if (self.disp_lp[0] and self.markers[p]=='o') or (self.disp_lp[1] and self.markers[p]=='v'):
                    if self.j == 10 and y[p] < 0:
                        ax.plot(x[p], y[p], marker=self.markers[p], color='white', mec='black')
                    else:
                        ax.plot(x[p], y[p], marker=self.markers[p], color=self.colors[p], label=self.labels[p])
                    if self.annotate:
                        ax.annotate(p, xy=(x[p],y[p]), xytext=(6,6), textcoords='offset pixels')
        if self.disp_witness:
            ax.axhline(y[41], linewidth=3, color='grey', label='witness 50')
            ax.axhline(y[42], linewidth=3, color='black', label='witness 100')
        if self.validation_color and self.j in {2,4}:
            #ax.patch.set_facecolor('red')
            #ax.patch.set_alpha(0.7)
            #sep_line = ax.axhline(2.0, linewidth=0.5, color='green')
            #ax.fill_between(x[:n], [2.0]*n, color='green', alpha=0.7)
            x0, y0, width, height = ax.dataLim.bounds
            ymin, ymax = ax.get_ylim()
            lim = height*2.0/(ymax-ymin)
            rect_g = Rectangle((x0,y0), width, 2.0-y0, edgecolor='green', facecolor='lightgreen', alpha=0.7)
            rect_r = Rectangle((x0,y0), width, height, edgecolor='red', facecolor='tomato', alpha=0.5)
            # x0, y0, width, height = ax.viewLim.bounds
            # print(ax.viewLim.bounds)
            # ymin, ymax = ax.get_ylim()
            # lim = height*2.0/(ymax-ymin)
            # rect_g = Rectangle((0,0), width+2*x0, height, edgecolor='green', facecolor='lightgreen')
            # rect_r = Rectangle((0,ymin), width+x0, height*4/5, edgecolor='red', facecolor='tomato')
            ax.add_patch(rect_r)
            ax.add_patch(rect_g)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return figure

if __name__ == "__main__":
    aux_frame = tk.Tk()
    graph_fig = Fig()
    aux_frame.title(graph_fig.output)

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
    cutoff.set(1000000)
    anno = tk.IntVar()
    anno.set(0)
    wit = tk.IntVar()
    wit.set(0)
    valcol = tk.IntVar()
    valcol.set(1)
    lp = tk.IntVar()
    lp.set(1)
    vo = tk.IntVar()
    vo.set(1)
    groups = []

    def update_plot():
        clear_frame(graph_frame)

        graph_fig.i = x.get()
        graph_fig.j = y.get()
        graph_fig.cutoff = cutoff.get()
        graph_fig.disp_lp = [bool(lp.get()), bool(vo.get())]
        for g in range(graph_fig.n_groups):
            graph_fig.disp_groups[g] = bool(groups[g].get())
        graph_fig.annotate = anno.get()
        graph_fig.disp_witness = wit.get()
        graph_fig.validation_color = valcol.get()

        fig = graph_fig.plot_graph()
        chart_type = FigureCanvasTkAgg(fig, graph_frame)
        chart_type.get_tk_widget().pack()
        cutoff.set(1000000)

    x_list = [["Opus",0], ["Speed of the ASV",2], ["Obstacle Heading",4], ["Theoretical dCPA",6], ["Detection Distance",9]]
    y_list = [["Time",1], ["Natural Collision Indic.",3], ["Logarithmic Collision Indic.",2],
              ["Offset Collision Indic.",4], ["Anticipation Inv Indic.",5], ["Anticipation Off Indic.",6],
              ["Anticipation Lin Indic.",7], ["Anticipation Exp Indic.",8], ["Real dCPA", 9], ["Crossing Distance", 10]]

    for w in x_list:
        tk.Radiobutton(frame1, variable=x, text=w[0], value=w[1], highlightthickness=0, command=update_plot).pack(fill='both')
    for w in y_list:
        tk.Radiobutton(frame2, variable=y, text=w[0], value=w[1], highlightthickness=0, command=update_plot).pack(fill='both')


    frame3_1 = tk.Frame(frame3)
    frame3_1.pack()
    tk.Label(frame3_1, text='Cutoff:').pack(side='left')
    tk.Entry(frame3_1, textvariable=cutoff, width=5).pack()
    tk.Button(frame3, text='Set Cutoff', command=update_plot).pack()
    tk.Checkbutton(frame3, text="Annotate", variable=anno, command=update_plot).pack()
    tk.Checkbutton(frame3, text="Display Witnesses", variable=wit, command=update_plot).pack()
    tk.Checkbutton(frame3, text="Validation Colors", variable=valcol, command=update_plot).pack()
    tk.Checkbutton(frame3, text="Display Non-LP", variable=lp, command=update_plot).pack()
    tk.Checkbutton(frame3, text="Display VO", variable=vo, command=update_plot).pack()

    for g in range(graph_fig.n_groups):
        groups.append(tk.IntVar())
        groups[g].set(1)
        tk.Checkbutton(frame3, text=f"Display Group {g+1}", variable=groups[g], command=update_plot).pack()


    update_plot()

    aux_frame.mainloop()
