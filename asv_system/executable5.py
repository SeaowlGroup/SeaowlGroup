#!/usr/bin/env python3

import roslaunch
import yaml
import numpy as np
import time
import datetime
from tkinter import *
import rospkg

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
        self.serial = serial
        self.opus = opus

    def graphic_interface(self):

        fenetre = Tk()
        fenetre.geometry("1000x800")
        fenetre.title("Parameters")
        fenetre.columnconfigure(0, weight=1)
        fenetre.columnconfigure(1, weight=1)
        fenetre.columnconfigure(2, weight=1)
        fenetre.rowconfigure(0, weight=1)
        fenetre.rowconfigure(1, weight=1)
        fenetre.rowconfigure(2, weight=1)

        ###################

        main_frame = Frame(fenetre, width=1000, height=600)
        main_frame.pack()
        #main_frame.geometry("1000x600")

        ###################

        seagull = PhotoImage(file="Seagull-USV.png")

        canvas = Canvas(main_frame, width=1000, height=600)
        canvas.create_image(500, 200, anchor=CENTER, image=seagull)
        canvas.grid(row=0, column=0, rowspan=3, columnspan=3)

        ###################

        l0 = LabelFrame(main_frame, text="ASV", padx=80, pady=40, bg='white')
        l0.grid(row=0, column=1)

        u_d_asv = DoubleVar()
        u_d_asv.set(5.0)
        Label(l0, text="Speed : ", bg='white').grid(row=0, column=0)
        Entry(l0, textvariable=u_d_asv, width=5, bg='whitesmoke').grid(row=0, column=1)

        lp = IntVar()
        lp.set(0)
        Label(l0, text="Local Planner : ", bg='white').grid(row=1, column=0)
        l01 = Frame(l0, bg='white')
        l01.grid(row=1, column=1)
        Radiobutton(l01, variable=lp, text="None", value=0, bg='white', relief='ridge').pack()
        Radiobutton(l01, variable=lp, text="Velocity Obstacles", value=1, bg='white').pack()

        ########################

        l1 = LabelFrame(main_frame, text='Obstacle ship', padx=80, pady=30, bg='white')
        l1.grid(row=1, column=1)

        heading = DoubleVar()
        heading.set(180.0)
        Label(l1, text="Heading : ", bg='white').grid(row=0, column=0)
        Entry(l1, textvariable=heading, width=5, bg='whitesmoke').grid(row=0, column=1)

        u_d = DoubleVar()
        u_d.set(5.0)
        Label(l1, text="Speed : ", bg='white').grid(row=1, column=0)
        Entry(l1, textvariable=u_d, width=5, bg='whitesmoke').grid(row=1, column=1)

        dcpa = DoubleVar()
        dcpa.set(0.0)
        Label(l1, text="dCPA : ", bg='white').grid(row=2, column=0)
        Entry(l1, textvariable=dcpa, width=5, bg='whitesmoke').grid(row=2, column=1)

        size = DoubleVar()
        size.set(8.0)
        Label(l1, text="Size : ", bg='white').grid(row=3, column=0)
        Entry(l1, textvariable=size, width=5, bg='whitesmoke').grid(row=3, column=1)

        prior = StringVar()
        prior.set("none")
        Label(l1, text="Priority status : ", bg='white').grid(row=4, column=0)
        l11 = Frame(l1, bg='white')
        l11.grid(row=4, column=1)
        Radiobutton(l11, variable=prior, text="None", value="none", bg='white').pack()
        Radiobutton(l11, variable=prior, text="Stand On", value="stand_on", bg='white').pack()
        Radiobutton(l11, variable=prior, text="Give Way", value="give_way", bg='white').pack()

        d_detec = DoubleVar()
        d_detec.set(100.0)
        Label(l1, text="Distance of detection : ", bg='white').grid(row=5, column=0)
        Entry(l1, textvariable=d_detec, width=5, bg='whitesmoke').grid(row=5, column=1)

        ########################
        rospack = rospkg.RosPack()
        file = f"{rospack.get_path('asv_system')}/output/{self.serial}.txt"

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

            print('=====================================')
            print('local planner : ', self.lp)
            print('size : ', self.size)
            print('heading : ', self.heading)
            print('u_d : ', self.u_d)
            print('dcpa : ', self.dcpa)
            print('prior : ', self.prior)
            print('d_detec : ', self.d_detec)
            print('=====================================')

            self.run()

            if self.opus <=1 :
                global f
                f = open(file, "r")
                content = f.readline()
                Label(out, text=content, anchor='nw', bg='white').pack(fill="both")

            content = f.readline()
            Label(out, text=content, anchor='nw', bg='white').pack(fill="both")




        bouton=Button(main_frame, text="Start Simulation",
                      command=register,
                      bd=6, width=15, height=2)
        bouton.config(font=('Batang', 35))
        bouton.grid(row=2, column=1)

        ########################

        Label(fenetre, text=f'Output: {file}', anchor='w').pack(fill='both')
        out = Frame(fenetre, bg='white')
        out.pack(fill="both", expand="yes")
        #Label(out, text='aaahahaha', anchor='nw', bg='white').pack(fill="both")

        ########################
        fenetre.mainloop()
        f.close()
        ########################

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
                     f'output_file:=$(find asv_system)/output/{self.serial}.txt',
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
        try :
            launch.start()
            launch.spin()
        except roslaunch.core.RLException :
            print('hum')
            return


if __name__ == "__main__":

    # Output parameters
    now = datetime.datetime.now()
    serial = now.strftime("%Y%m%d%H%M%S")[2:]

    scenar = Scenario(serial, 0)

    scenar.graphic_interface()
