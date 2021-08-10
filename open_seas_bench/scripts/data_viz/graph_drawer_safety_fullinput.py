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
        self.dcpamin = -80.
        self.dcpamax = 80.
        self.ddetectmin = 100
        self.ddetectmax = 500
        self.delta = np.array([1,1,20,1,10,100])
        self.i = 0
        self.j = 1

        self.lab = ['OPUS', 'ASV SPEED', 'HEADING', 'OBSTACLE SPEED', 'DCPA', 'DETECTION DISTANCE']

        self.labels = []
        self.colors = []
        self.markers = []
        self.total = 0
        self.count = 0

        f1 = open(self.input,'r')
        f1.readline()
        self.lps = []
        self.sits = []
        self.idata = []
        self.odata = []
        for line in f1:
            content = line.split()
            self.idata.append([float(content[0]),float(content[2]),float(content[4]),float(content[5]),float(content[6]),float(content[9])])
            if content[3] == "True":
                self.lps.append(1)
            else:
                self.lps.append(0)
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
            self.labels.append(f"{content[1]}")
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
        self.disp_witness = False
        self.validation_color = True

    def set_security_and_performances_classes(self):
        f = open(self.output, 'r')
        f.readline()
        self.groups = []
        for line in f:
            content = line.split()
            if float(content[4]) > .2: # dangerous security indicator
                i=3
                self.colors.append('r')
            elif float(content[4]) >= -0.2: # insecure
                if float(content[6]) > self.anticip_cutoff: # 
                    i=2
                    self.colors.append('y')
                else:
                    i=1
                    self.colors.append('g')
            else : # safe
                i=0
                self.colors.append('b')
            if float(content[1]) > self.time_cutoff: # too slow
                j=1
                self.markers.append('X')
            else : # performant
                j=0
                self.markers.append('.')
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
        ax.set_title("Results of the Simulation")
        ax.set_xlabel(self.lab[self.i])
        ax.set_ylabel(self.lab[self.j])

        p = 0
        f = open(self.input,'r')
        
        x = self.idata[:,self.i]
        y = self.idata[:,self.j]

        n = min(len(x), len(y))
        self.count = 0
        r = (2*np.random.rand(n,2)-1)/4
        print(self.delta[self.i]/4,self.delta[self.j])
        for p in range(n):
            if self.disp_groups[self.groups[p]] and self.disp_lp[self.lps[p]] and self.disp_sit[self.sits[p]]:
                self.count += 1 
                ax.plot(x[p]+r[p,0]*self.delta[self.i], y[p]+r[p,1]*self.delta[self.j], marker=self.markers[p], color=self.colors[p])
                ax.plot(x[p], y[p], marker='x', color='k')        
                if self.annotate:
                    ax.annotate(p, xy=(x[p],y[p]), xytext=(6,6), textcoords='offset pixels')
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
    x.set(0)
    y.set(0)
    dcpamin = tk.DoubleVar()
    dcpamin.set(-80)
    dcpamax = tk.DoubleVar()
    dcpamax.set(80)
    ddetectmin = tk.DoubleVar()
    ddetectmin.set(100)
    ddetectmax = tk.DoubleVar()
    ddetectmax.set(500)
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

    def update_plot():
        clear_frame(graph_frame)

        graph_fig.i = x.get()
        graph_fig.j = y.get()
        graph_fig.dcpamin = dcpamin.get()
        graph_fig.dcpamax = dcpamax.get()
        graph_fig.ddetectmax = ddetectmax.get()
        graph_fig.ddetectmin = ddetectmin.get()
        graph_fig.disp_lp = [bool(lp.get()), bool(vo.get())]
        graph_fig.disp_sit = [bool(ov1.get()), bool(ov2.get()), bool(cr.get()),bool(cl.get()),bool(ho.get())]
        for g in range(graph_fig.n_groups):
            graph_fig.disp_groups[g] = bool(groups[g].get())
        graph_fig.annotate = anno.get()
        graph_fig.disp_witness = wit.get()
        graph_fig.validation_color = valcol.get()

        fig = graph_fig.plot_graph()
        chart_type = FigureCanvasTkAgg(fig, graph_frame)
        chart_type.get_tk_widget().pack()

    x_list = [["Opus",0], ["ASV Speed",1], ["Obstacle Heading",2], ["Obstacle Speed",3], ["Theoretical dCPA",4], ["Detection Distance",5]]
    y_list = [["Opus",0], ["ASV Speed",1], ["Obstacle Heading",2], ["Obstacle Speed",3], ["Theoretical dCPA",4], ["Detection Distance",5]]
    

    for w in x_list:
        tk.Radiobutton(frame1, variable=x, text=w[0], value=w[1], highlightthickness=0).pack(fill='both')
    for w in y_list:
        tk.Radiobutton(frame2, variable=y, text=w[0], value=w[1], highlightthickness=0).pack(fill='both')


    frame3_1 = tk.Frame(frame3)
    tk.Button(frame3, text='Update', command=update_plot).pack()
    frame3_1.pack()
    #tk.Label(frame3_1, text='DCPA_MIN:', width=5).pack(side='left')
    #tk.Entry(frame3_1, textvariable=dcpamin, width=5).pack()
    #tk.Label(frame3_1, text='DCPA_MAX:', width=5).pack(side='left')
    #tk.Entry(frame3_1, textvariable=dcpamax, width=5).pack()
    #tk.Label(frame3_1, text='D_DETECT_MIN:',width=5).pack(side='left')
    #tk.Entry(frame3_1, textvariable=ddetectmin, width=5).pack()
    #tk.Label(frame3_1, text='D_DETECT_MAX:',width=5).pack(side='left')
    #tk.Entry(frame3_1, textvariable=ddetectmax, width=5).pack()
    tk.Checkbutton(frame3, text="Annotate", variable=anno).pack()
    #tk.Checkbutton(frame3, text="Display Non-LP", variable=lp).pack()
    #tk.Checkbutton(frame3, text="Display VO", variable=vo).pack()
    tk.Checkbutton(frame3, text="Display Overtaking", variable=ov1).pack()
    tk.Checkbutton(frame3, text="Display Overtaken", variable=ov2).pack()
    tk.Checkbutton(frame3, text="Display Crossing right", variable=cr).pack()
    tk.Checkbutton(frame3, text="Display Crossing left", variable=cl).pack()
    tk.Checkbutton(frame3, text="Display Head on", variable=ho).pack()


    # for g in range(graph_fig.n_groups):
    l1 = ['Safe', 'Not quite Safe', 'Not quite Dangerous', 'Dangerous']
    l2 = ['Fast', 'Slow']
    for i in range(4):
        for j in range(2):
            groups.append(tk.IntVar())
    for i in range(4):
        for j in range(2):
            g = i + 4*j
            groups[g].set(0)
            tk.Checkbutton(frame3, text=f"Display {l1[i]}/{l2[j]}", variable=groups[g]).pack()

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
