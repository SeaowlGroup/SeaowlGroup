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
        self.bins = [range(35000),
                     [3, 5, 8, 10, 15, 20, 25, 30, 35],
                     [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 360], 
                     [3, 5, 8, 10, 15, 20, 25, 30, 35],
                     [-80, -50, -20, -10, 0, 10, 20, 50, 80, 110],
                     [50,100,200,500, 800]]
        self.x = 0
        self.y = 1
        self.lab = ['OPUS', 'ASV SPEED (kn)', 'HEADING (°)', 'OBSTACLE SPEED (kn)', 'DCPA (m)', 'DETECTION DISTANCE (m)']
        self.gplab = []
        self.colors = ['green','yellow','orange','red','darkgreen','gold','darkorange','darkred']
        self.markers = []
        self.total = 0
        self.count = 0
        self.graphType = 0

        f1 = open(self.input,'r')
        f1.readline()
        self.sits = []
        self.idata = []
        self.odata = []
        for line in f1:
            content = line.split()
            self.idata.append([float(content[0]),float(content[2]),float(content[4]),float(content[5]),float(content[6]),float(content[9])])
            if content[1] == "OVERTAKING":
                self.sits.append(0)
            elif content[1] == "OVERTAKEN":
                self.sits.append(1)
            elif content[1] == "CROSSING_RIGHT":
                self.sits.append(2)
            elif content[1] == "CROSSING_LEFT":
                self.sits.append(3)
            elif content[1] == "HEAD_ON":
                self.sits.append(4)
            else:
                raise("Unexpected")
            self.total += 1
        self.idata = np.array(self.idata)

        f1.close()
        self.groups = []

        self.time_cutoff = 80.
        self.anticip_cutoff = 0.12
        self.security_and_performances_classes = np.zeros((4,2))
        self.set_security_and_performances_classes()

        self.n_groups = 8
        self.cutoff = 1000000
        self.disp_lp = [True]*2
        self.disp_groups = [True]*self.n_groups
        self.disp_sit = [True]*5
        self.annotate = False

    def set_security_and_performances_classes(self):
        f = open(self.output, 'r')
        f.readline()
        self.groups = []
        for line in f:
            content = line.split()
            if float(content[4]) > .2: # dangerous security indicator
                i=3
            elif float(content[4]) >= -0.2: # insecure
                if float(content[6]) > self.anticip_cutoff: # 
                    i=2
                else:
                    i=1
            else : # safe
                i=0
            if float(content[1]) > self.time_cutoff: # too slow
                j=1
            else : # performant
                j=0
            self.groups.append(i + j*4)
            self.security_and_performances_classes[i,j] += 1
        f.close()

        prop = self.security_and_performances_classes
        tot = np.sum(prop)

        print("Total number : ", tot)
        print(f"SEC\PERF             |   Performant\t | Insufficient")
        print(f"Safe                 |   {round(100*prop[0, 0]/tot, 1)}%\t | {round(100*prop[0, 1]/tot, 1)}%      |")
        print(f"Not quite safe       |   {round(100*prop[1, 0]/tot, 1)}%\t | {round(100*prop[1, 1]/tot, 1)}%      |")
        print(f"Not quite dangerous  |   {round(100*prop[2, 0]/tot, 1)}%\t | {round(100*prop[2, 1]/tot, 1)}%      |")
        print(f"Dangerous            |   {round(100*prop[3, 0]/tot, 1)}%\t | {round(100*prop[3, 1]/tot, 1)}%      |")

    def plot_graph(self) :

        figure = plt.Figure(figsize=(8,8), dpi=100)

        ax = figure.add_subplot(111)
        ax.set_title("Results of the Simulation from the Open Seas Bench")
        ax.set_xlabel(self.lab[self.x])
        ax.set_ylabel(self.lab[self.y])

        p = 0
        f = open(self.input,'r')
        
        x = self.idata[:,self.x]
        y = self.idata[:,self.y]

        n = min(len(x), len(y))
        self.count = 0
        r = (2*np.random.rand(n,2)-1)/4
        for p in range(n):
            if self.disp_groups[self.groups[p]] and self.disp_sit[self.sits[p]]:
                self.count += 1 
                ax.plot(x[p]+r[p,0]*self.delta[self.x], y[p]+r[p,1]*self.delta[self.y], marker=self.markers[p], color=self.colors[p])
                ax.plot(x[p], y[p], marker='x', color='k')        
                if self.annotate:
                    ax.annotate(p, xy=(x[p],y[p]), xytext=(6,6), textcoords='offset pixels')
        print("Count : ", self.count, " / ", self.total)
        print("Count : ", p, " / ", self.total)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return figure
    
    def plot_hist1d(self) :

        figure = plt.Figure(figsize=(8,8), dpi=100)

        ax = figure.add_subplot(111)
        ax.set_title("Results of the Simulation from the Open Seas Bench")
        ax.set_xlabel(self.lab[self.x])
        ax.set_ylabel("Number")
        
        n = len(self.idata)
        m = len(self.disp_groups)
        #print(f'disp gp : {self.disp_groups}')
        x = m*[None]
        for k in range(m):
            x[k] = []
        colors = [self.colors[k] for k in self.disp_groups]
        gplab = [self.gplab[k] for k in self.disp_groups]

        for p in range(n):
            if self.groups[p] in self.disp_groups and self.disp_sit[self.sits[p]]:
                #print(f'gp: {self.groups[p]}')
                #print(f'index : {self.disp_groups.index(self.groups[p])}')
                x[self.disp_groups.index(self.groups[p])].append(self.idata[p,self.x])
            #print(x)
            #input()
        ax.hist(x,bins = self.bins[self.x],color = colors, label = gplab)
        print("Count : ", self.count, " / ", self.total)
        print("Count : ", p, " / ", self.total)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return figure
    
    def plot_hist2d(self) :

        figure = plt.Figure(figsize=(8,8), dpi=100)

        ax = figure.add_subplot(111)
        ax.set_title("Results of the Simulation from the Open Seas Bench")
        ax.set_xlabel(self.lab[self.x])
        ax.set_ylabel(self.lab[self.y])
        
        n = len(self.idata)
        #print(f'disp gp : {self.disp_groups}')
        x = []
        y = []
        for p in range(n):
            if self.groups[p] in self.disp_groups and self.disp_sit[self.sits[p]]:
                #print(f'gp: {self.groups[p]}')
                #print(f'index : {self.disp_groups.index(self.groups[p])}')
                x.append(self.idata[p,self.x])
                y.append(self.idata[p,self.y])
            #print(x)
            #input()
        _, _, _, im = ax.hist2d(x,y,bins = [self.bins[self.x],self.bins[self.y]], cmap = plt.get_cmap('seismic'))
        figure.colorbar(im)
        print("Count : ", self.count, " / ", self.total)
        print("Count : ", p, " / ", self.total)

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
    x.set(2)
    y.set(1)
    anno = tk.IntVar()
    anno.set(0)
    valcol = tk.IntVar()
    valcol.set(1)
    ov1 = tk.IntVar()
    ov1.set(1)
    ov2 = tk.IntVar()
    ov2.set(1)
    cr = tk.IntVar()
    cr.set(1)
    cl = tk.IntVar()
    cl.set(1)
    ho = tk.IntVar()
    ho.set(1)
    groups = []
    graphType = tk.IntVar()
    graphType.set(1)

    def update_plot():
        clear_frame(graph_frame)

        graph_fig.x = x.get()
        graph_fig.y = y.get()
        graph_fig.disp_sit = [bool(ov1.get()), bool(ov2.get()), bool(cr.get()),bool(cl.get()),bool(ho.get())]
        graph_fig.disp_groups = []
        for g in range(graph_fig.n_groups):
            if bool(groups[g].get()):
                graph_fig.disp_groups.append(g)
        graph_fig.annotate = anno.get()
        graph_fig.graphType = graphType.get()
        if graph_fig.graphType == 1:
            fig = graph_fig.plot_hist1d()
        elif graph_fig.graphType == 2:
            fig = graph_fig.plot_hist2d()
        else: 
            raise("error")
        chart_type = FigureCanvasTkAgg(fig, graph_frame)
        chart_type.get_tk_widget().pack()

    x_list = [["Opus",0], ["ASV Speed",1], ["Obstacle Heading",2], ["Obstacle Speed",3], ["Theoretical dCPA",4], ["Detection Distance",5]]
    y_list = [["Opus",0], ["ASV Speed",1], ["Obstacle Heading",2], ["Obstacle Speed",3], ["Theoretical dCPA",4], ["Detection Distance",5]]
    

    for w in x_list:
        tk.Radiobutton(frame1, variable=x, text=w[0], value=w[1], highlightthickness=0).pack(fill='both')
    for w in y_list:
        tk.Radiobutton(frame2, variable=y, text=w[0], value=w[1], highlightthickness=0).pack(fill='both')

    tk.Label(frame3, text='Graph Type').pack()
    tk.Radiobutton(frame3, variable=graphType, text='Histogram1D', value=1, highlightthickness=0).pack()
    tk.Radiobutton(frame3, variable=graphType, text='Histogram2D', value=2, highlightthickness=0).pack()
    tk.Checkbutton(frame3, text="Annotate", variable=anno).pack()
    tk.Checkbutton(frame3, text="Display Overtaking", variable=ov1).pack()
    tk.Checkbutton(frame3, text="Display Overtaken", variable=ov2).pack()
    tk.Checkbutton(frame3, text="Display Crossing right", variable=cr).pack()
    tk.Checkbutton(frame3, text="Display Crossing left", variable=cl).pack()
    tk.Checkbutton(frame3, text="Display Head on", variable=ho).pack()

    l1 = ['Safe', 'Not quite Safe', 'Not quite Dangerous', 'Dangerous']
    l2 = ['Fast', 'Slow']
    for i in range(4):
        for j in range(2):
            groups.append(tk.IntVar())
    for j in range(2):
        for i in range(4):
            g = i + 4*j
            groups[g].set(0)
            tk.Checkbutton(frame3, text=f"Display {l1[i]}/{l2[j]}", variable=groups[g]).pack()
            graph_fig.gplab.append(f"{l1[i]}/{l2[j]}")
    
    tk.Button(frame3, text='Update', command=update_plot).pack()

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
