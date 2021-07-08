#!/usr/bin/env python3

import roslaunch
import yaml
import numpy as np
import time
import datetime
from tkinter import *
import rospkg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def clear_frame(frame):
    for w in frame.winfo_children():
        w.destroy()

class Scenario(object):

    def __init__(self, serial='000000', opus=0):
        # ASV related attributes
        self.u_d_asv = None
        self.lp = None
        self.true_heading_asv = 0.0
        self.t_sim = 30
        # Obstacle related attributes
        self.heading = None
        self.u_d = None
        self.dcpa = None
        self.size = None
        self.prior = None
        self.d_detec = None
        self.t_collision = 15
        # Output
        rospack = rospkg.RosPack()
        self.input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
        self.output = f"{rospack.get_path('asv_system')}/output/{serial}.txt"
        self.opus = opus

    def graphic_interface(self):

        fenetre = Tk()
        screen_width = fenetre.winfo_screenwidth()
        screen_height = fenetre.winfo_screenheight()
        w_big = int(screen_width*5/8)
        w_small = int(screen_width*3/8)
        h_unit = int(screen_height/3)
        #fenetre.geometry("1600x900")
        fenetre.geometry(f"{screen_width}x{screen_height}")
        fenetre.title("Bench Test")
        fenetre.iconphoto(False, PhotoImage(file='icon.png'))
        fenetre.configure(bg='gainsboro')
        fenetre.columnconfigure(0, weight=1)
        fenetre.columnconfigure(1, weight=1)
        fenetre.columnconfigure(2, weight=1)
        fenetre.rowconfigure(0, weight=1)
        fenetre.rowconfigure(1, weight=1)
        fenetre.rowconfigure(2, weight=1)

        ###################

        main_frame = Frame(fenetre, width=w_big, height=screen_height)
        main_frame.pack(side=LEFT)
        first_frame = Frame(main_frame, width=w_big, height=2*h_unit)
        first_frame.pack()

        Label(main_frame, text=f'Output: {self.output}', anchor=W, bg='lightslategrey', fg='white').pack(fill='x')
        second_frame = Frame(main_frame, bg='white', width=w_big, height=h_unit)
        second_frame.pack_propagate(0)
        second_frame.pack(fill="both", expand="yes")

        aux_frame = Frame(fenetre, width=w_small, height=screen_height, bg='gainsboro')
        aux_frame.pack(fill='both')
        Label(aux_frame, text='Plot data', bg='lightslategrey', fg='white', width=100).pack()

        ###################
        ###################

        seagull = PhotoImage(file="Seagull-USV.png")

        canvas = Canvas(first_frame, width=w_big, height=2*h_unit)
        canvas.create_image(w_big/2, 2*h_unit/3, anchor=CENTER, image=seagull)
        canvas.grid(row=0, column=0, rowspan=3, columnspan=3)

        ###################

        l0 = LabelFrame(first_frame, text="ASV", padx=80, pady=40, bg='white')
        l0.grid(row=0, column=1)

        u_d_asv = DoubleVar()
        u_d_asv.set(5.0)
        Label(l0, text="Speed : ", bg='white', anchor=E).grid(row=0, column=0, sticky="nsew")
        Entry(l0, textvariable=u_d_asv, width=5, bg='whitesmoke').grid(row=0, column=1)

        lp = IntVar()
        lp.set(0)
        Label(l0, text="Local Planner : ", bg='white', anchor=E).grid(row=1, column=0, sticky="nsew")
        l01 = Frame(l0, bg='white')
        l01.grid(row=1, column=1)
        Radiobutton(l01, variable=lp, text="None", value=0, bg='white', anchor=W, highlightthickness=0).pack(fill='both')
        Radiobutton(l01, variable=lp, text="Velocity Obstacles", value=1, bg='white', anchor=W, highlightthickness=0).pack(fill='both')

        ########################

        l1 = LabelFrame(first_frame, text='Obstacle ship', padx=80, pady=30, bg='white')
        l1.grid(row=1, column=1)

        heading = DoubleVar()
        heading.set(180.0)
        Label(l1, text="Heading : ", bg='white', anchor=E).grid(row=0, column=0, sticky="nsew")
        Entry(l1, textvariable=heading, width=5, bg='whitesmoke').grid(row=0, column=1)

        u_d = DoubleVar()
        u_d.set(5.0)
        Label(l1, text="Speed : ", bg='white', anchor=E).grid(row=1, column=0, sticky="nsew")
        Entry(l1, textvariable=u_d, width=5, bg='whitesmoke').grid(row=1, column=1)

        dcpa = DoubleVar()
        dcpa.set(0.0)
        Label(l1, text="dCPA : ", bg='white', anchor=E).grid(row=2, column=0, sticky="nsew")
        Entry(l1, textvariable=dcpa, width=5, bg='whitesmoke').grid(row=2, column=1)

        size = DoubleVar()
        size.set(8.0)
        Label(l1, text="Size : ", bg='white', anchor=E).grid(row=3, column=0, sticky="nsew")
        Entry(l1, textvariable=size, width=5, bg='whitesmoke').grid(row=3, column=1)

        prior = StringVar()
        prior.set("none")
        Label(l1, text="Priority status : ", bg='white', anchor=E).grid(row=4, column=0, sticky="nsew")
        l11 = Frame(l1, bg='white')
        l11.grid(row=4, column=1)
        Radiobutton(l11, variable=prior, text="None", value="none", bg='white', anchor=W, highlightthickness=0).pack(fill='both')
        Radiobutton(l11, variable=prior, text="Stand On", value="stand_on", bg='white', anchor=W, highlightthickness=0).pack(fill='both')
        Radiobutton(l11, variable=prior, text="Give Way", value="give_way", bg='white', anchor=W, highlightthickness=0).pack(fill='both')

        d_detec = DoubleVar()
        d_detec.set(100.0)
        Label(l1, text="Distance of detection : ", bg='white', anchor=E).grid(row=5, column=0, sticky="nsew")
        Entry(l1, textvariable=d_detec, width=5, bg='whitesmoke').grid(row=5, column=1)

        ########################

        def register():

            self.u_d_asv = u_d_asv.get()
            self.lp = (lp.get() == 1)
            self.heading = heading.get()
            self.u_d = u_d.get()
            self.dcpa = dcpa.get()
            self.size = size.get()
            self.prior = prior.get()
            self.d_detec = d_detec.get()

            #fenetre.destroy()
            self.opus += 1

            self.write_input()

            self.run()
            try:
                if self.opus == 1 :
                    global f
                    f = open(self.output, "r")
                    content = f.readline().rstrip('\n')
                    Label(second_frame, text=content, anchor=NW, bg='white').pack(fill="x")

                content = f.readline().rstrip('\n')
                Label(second_frame, text=content, anchor=NW, bg='white').pack(fill="x")
            except FileNotFoundError:
                self.opus-=1
                #clear input
                pass

            update_plot()


        bouton=Button(first_frame, text="Start Simulation",
                      command=register,
                      bd=6, width=15, height=2)
        bouton.config(font=('Batang', 35))
        bouton.grid(row=2, column=1)

        ########################
        ########################

        graph_frame = Frame(aux_frame, width=w_small, height=2*h_unit, bg='whitesmoke')
        graph_frame.pack(fill='both', expand='yes')
        Label(graph_frame, text='No data', bg='whitesmoke', anchor=CENTER).pack(fill='both', pady=h_unit-10)

        #fig = plot_graph(0, 2, w_small/100, 2*h_unit/100)
        #chart_type = FigureCanvasTkAgg(fig, aux_frame)
        #graph = chart_type.get_tk_widget().pack()


        f1 = Frame(aux_frame, bg='gainsboro')
        f2 = Frame(aux_frame, bg='gainsboro')
        f1.pack(side=LEFT, expand='yes')
        f2.pack(side=LEFT, expand='yes')
        Label(f1, text='X :', bg='gainsboro').pack()
        Label(f2, text='Y :', bg='gainsboro').pack()

        x = IntVar()
        y = IntVar()
        x_list = [["Opus",0], ["ASV Speed",1], ["Obstacle Heading",3]]
        y_list = [["Natural Collision Indic.",2], ["Logarithmic Collision Indic.",1], ["Anticipation Indic.",4]]
        x.set(0)
        y.set(2)

        def update_plot():
            if self.opus >= 1:
                clear_frame(graph_frame)
                fig = self.plot_graph(x.get(), y.get(), w_small/100, 2*h_unit/100)
                #FigureCanvasTkAgg(fig, aux_frame).get_tk_widget().pack()
                chart_type = FigureCanvasTkAgg(fig, graph_frame)
                chart_type.get_tk_widget().pack()

        for w in x_list:
            Radiobutton(f1, variable=x, text=w[0], value=w[1], bg='gainsboro', anchor=W, highlightthickness=0, command=update_plot).pack(fill='both')
        for w in y_list:
            Radiobutton(f2, variable=y, text=w[0], value=w[1], bg='gainsboro', anchor=W, highlightthickness=0, command=update_plot).pack(fill='both')

        ########################
        ########################
        fenetre.mainloop()
        try:
            f.close()
        except NameError:
            pass
        ########################

    def write_input(self):

        param = [self.u_d_asv,
                 self.lp,
                 self.heading,
                 self.u_d,
                 self.dcpa,
                 self.size,
                 self.prior,
                 self.d_detec]

        f = open(self.input,'a')

        if self.opus == 1:
            f.write(f'OPUS    U_D_ASV    LOC_PLAN    HEADING    U_D    DCPA    SIZE    PRIOR    D_DETEC\n')
        f.write(f'{self.opus}')
        for p in param:
            f.write(f'    {p}')
        f.write('\n')
        f.close()

        print('=====================================')
        print('local planner : ', self.lp)
        print('size : ', self.size)
        print('heading : ', self.heading)
        print('u_d : ', self.u_d)
        print('dcpa : ', self.dcpa)
        print('prior : ', self.prior)
        print('d_detec : ', self.d_detec)
        print('=====================================')

    def run(self):

        # UUID
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        # ASV parameters
        calc_heading_asv = (90-self.true_heading_asv)*np.pi/180
        initial_state_asv = [0.,0.,calc_heading_asv, self.u_d_asv,0.,0.]
        #Trajectory
        waypoints_asv = [[0.,0.],
                         [self.t_sim*self.u_d_asv*np.cos(calc_heading_asv),
                          self.t_sim*self.u_d_asv*np.sin(calc_heading_asv)]]

        # Creation of the launch files
        cli_args0 = ['asv_system', 'main_launch2.launch',
                     f'initial_state:={initial_state_asv}',
                     f'waypoints:={waypoints_asv}',
                     f'u_d:={self.u_d_asv}',
                     f'use_vo:={self.lp}',
                     f'output_file:={self.output}',
                     f'opus:={self.opus}']
        roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
        roslaunch_args0 = cli_args0[2:]

        cli_args1 = ['asv_obstacle_tracker', 'obst_simplified.launch',
                     f'prior:=[{self.prior}]',
                     f'size:=[{self.size}]',
                     f'heading:=[{self.heading}]',
                     f'u_d:=[{self.u_d}]',
                     f't_collision:=[{self.t_collision}]',
                     f'd_detection:=[{self.d_detec}]',
                     f'dcpa:=[{self.dcpa}]',
                     f'initial_state_asv:={initial_state_asv}']
        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
        roslaunch_args1 = cli_args1[2:]
        launch_files = [(roslaunch_file0, roslaunch_args0), (roslaunch_file1, roslaunch_args1)]

        launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        launch.start()
        launch.spin()

    def plot_graph(self, i, j, width, height):

        x = []
        y = []
        xlab = ['OPUS', 'U_D_ASV', 'LOC_PLAN', 'HEADING', 'U_D', 'DCPA', 'SIZE', 'PRIOR', 'D_DETEC']
        ylab = ['LOG_COL', 'NAT_COL', 'OFFSET_LOG', 'ANTICIPATION']
        labels = []
        colors = []
        f1 = open(self.input,'r')
        f2 = open(self.output,'r')
        f1.readline()
        f2.readline()

        for line in f1:
            content = line.split()
            x.append(float(content[i]))
            labels.append('Velocity Obstacles' if content[2] == "True" else 'No LP')
            colors.append('orange' if content[2] == "True" else 'blue')
        for line in f2:
            content = line.split()
            y.append(float(content[j]))
        f1.close()
        f2.close()

        figure = plt.Figure(figsize=(width,height), dpi=100)

        ax = figure.add_subplot(111)
        ax.set_title("Results of the Simulation")
        ax.set_xlabel(xlab[i])
        ax.set_ylabel(ylab[j-1])

        for p in range(1,len(x)):
            ax.plot(x[p], y[p], 'o', color=colors[p], label=labels[p])

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return figure


if __name__ == "__main__":

    # Output parameters
    now = datetime.datetime.now()
    serial = now.strftime("%Y%m%d%H%M%S")[2:]

    scenar = Scenario(serial, 0)

    scenar.graphic_interface()
