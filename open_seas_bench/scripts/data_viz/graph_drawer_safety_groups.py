#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospkg
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle

COLOR = ['b', 'orange', 'crimson', 'purple', 'cyan', 'magenta', 'gold']

def clear_frame(frame):
    for w in frame.winfo_children():
        w.destroy()

class Fig(object):

    def __init__(self, input, output):
        self.input = input
        self.output = output

        self.i = 0
        self.j = 1

        self.xlab = ['OPUS', 'CLASS', 'U_D_ASV', 'LOC_PLAN', 'HEADING', 'U_D', 'DCPA', 'SIZE', 'PRIOR', 'D_DETEC']
        self.ylab = ['TIME', 'LOG_COL', 'NAT_COL', 'OFFSET_LOG', 'ANTICIPATION_ACC', 'ANTICIPATION_OMEGA',
                     'ANTICIPATION_R', 'AGG_ACC', 'AGG_OMEGA', 'AGG_R', 'DCPA', 'CROSSING_DIST', 'ANT_TIME', 'AGG_TIME']

        self.labels = []
        self.colors = []
        self.markers = []
        self.total = 0
        self.count = 0

        f1 = open(self.input,'r')
        f1.readline()

        for line in f1:
            self.total += 1
            content = line.split()
            #self.groups.append(int(content[10]))
            self.colors.append(COLOR[int(content[10])])
            if content[3] == "True":
                self.labels.append('Velocity Obstacles')
                self.markers.append('v')
            else:
                self.labels.append('No LP')
                self.markers.append('o')

            self.labels[-1]+=f"_{content[1]}"

        f1.close()
        self.groups = []

        self.time_cutoff = 80.
        self.anticip_cutoff = 0.12
        self.security_and_performances_classes = np.zeros((3,2))
        self.set_security_and_performances_classes()

        self.n_groups = 12
        self.cutoff = 1000000
        self.disp_lp = [True]*2
        self.disp_groups = [True]*self.n_groups
        self.annotate = False
        self.disp_witness = False
        self.validation_color = True

    def set_security_and_performances_classes(self):
        f = open(self.output, 'r')
        f.readline()
        self.groups = []
        for line in f:
            content = line.split()
            if float(content[4]) > 0.: # dangerous security indicator
                i=2
            elif float(content[4]) >= -0.2: # insecure
                i=1
            else : # safe
                i=0

            if float(content[1]) > self.time_cutoff: # too slow
                j=1
            else : # performant
                j=0

            if float(content[6]) > self.anticip_cutoff: # too slow
                k=1
            else : # performant
                k=0
            self.groups.append(i + j*3 + k*6)
            self.security_and_performances_classes[i,j] += 1
        f.close()

        prop = self.security_and_performances_classes
        tot = np.sum(prop)

        print("Total number : ", tot)
        print(f"SEC\PERF    Performant  Insufficient")
        print(f"Safe      |   {round(100*prop[0, 0]/tot, 1)}%    | {round(100*prop[0, 1]/tot, 1)}%      |")
        print(f"Insecure  |   {round(100*prop[1, 0]/tot, 1)}%    | {round(100*prop[1, 1]/tot, 1)}%      |")
        print(f"Dangerous |   {round(100*prop[2, 0]/tot, 1)}%    | {round(100*prop[2, 1]/tot, 1)}%      |")

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
        self.count = 0
        for p in range(n):
            #if p!=0 :
            if y[p] < self.cutoff and self.disp_groups[self.groups[p]]:
                if (self.disp_lp[0] and self.markers[p]=='o') or (self.disp_lp[1] and self.markers[p]=='v'):
                    self.count += 1
                    if self.j == 12 and y[p] < 0:
                        ax.plot(x[p], y[p], marker=self.markers[p], color='white', mec='black')
                    else:
                        ax.plot(x[p], y[p], marker=self.markers[p], color=self.colors[p], label=self.labels[p])
                    if self.annotate:
                        ax.annotate(p, xy=(x[p],y[p]), xytext=(6,6), textcoords='offset pixels')
        print("Count : ", self.count, " / ", self.total)
        if self.validation_color and self.j in {2,4}:
            x0, y0, width, height = ax.viewLim.bounds
            lim = 0.0
            rect_g = Rectangle((x0,y0), width, np.maximum(lim-y0, 0.0), edgecolor='green', facecolor='lightgreen', alpha=0.5)
            rect_r = Rectangle((x0,y0), width, height, edgecolor='red', facecolor='tomato', alpha=0.3)
            ax.add_patch(rect_r)
            ax.add_patch(rect_g)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return figure

def gui(name):
    title = name.split('.')[0]
    aux_frame.title(title)

    input = f"{rospack.get_path('open_seas_bench')}/input/{name}"
    output = f"{rospack.get_path('open_seas_bench')}/output/{name}"
    graph_fig = Fig(input=input, output=output)

    graph_frame = tk.Frame(aux_frame, bg='white')
    graph_frame.grid(row=1, rowspan=2, column=1)

    frame1 = tk.Frame(aux_frame, bg='gainsboro', padx=5)
    frame2 = tk.Frame(aux_frame, bg='gainsboro')
    frame3 = tk.Frame(aux_frame, bg='gainsboro', padx=20)
    frame1.grid(row=1, column=0)
    frame2.grid(row=2, column=0)
    frame3.grid(row=1, column=2)
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
              ["Offset Collision Indic.",4], ["Anticipation Acc Indic.",5], ["Anticipation Omega Indic.",6],
              ["Anticipation R Indic.",7], ["Agglutination Acc Indic.",8], ["Agglutination Omega Indic.",9],
              ["Agglutination R Indic.",10], ["Real dCPA", 11], ["Crossing Distance", 12],
              ["Anticipation Time",13], ["Agglutination Time",14]]


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

    # for g in range(graph_fig.n_groups):
    l1 = ['Safe', 'Insecure', 'Dangerous']
    l2 = ['Fast', 'Slow']
    l3 = ['Anticipated', 'Not_anticipated']
    for i in range(3):
        for j in range(2):
            for k in range(2):
                groups.append(tk.IntVar())
    for i in range(3):
        for j in range(2):
            for k in range(2):
                g = i + 3*j + 6*k
                groups[g].set(1)
                # tk.Checkbutton(frame3, text=f"Display Group {g+1}", variable=groups[g], command=update_plot).pack()
                tk.Checkbutton(frame3, text=f"Display {l1[i]}/{l2[j]}/{l3[k]}", variable=groups[g], command=update_plot).pack()


    update_plot()

    aux_frame.mainloop()

if __name__ == "__main__":
    aux_frame = tk.Tk()

    rospack = rospkg.RosPack()
    filepath = tk.filedialog.askopenfilename(title="Load a file :",filetypes=[('txt files','.txt'),('all files','.*')],
                                             initialdir=f"{rospack.get_path('open_seas_bench')}/output/")

    if filepath:
        name = filepath.split('/')[-1]
        gui(name)
    else:
        print("No file chosen")
