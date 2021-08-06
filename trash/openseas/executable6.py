#!/usr/bin/env python3

import roslaunch
import numpy as np
import time
import datetime
import pandas as pd
import rospkg
import tkinter as tk
from tkinter.filedialog import askopenfilename

def run(serial, input_file, parameter_file) :
    df = pd.read_excel(parameter_file,header=0)
    param = df.to_numpy()

    # UUID
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # ASV parameters
    calc_heading_asv = np.pi/2
    # u_d_asv = 5.0
    # initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
    t_sim = 75
    t_collision = 45
    #Trajectory
    # waypoints_asv = [[0.,0.],
    #                  [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]

    for opus in np.arange(len(param)):
        class_scen = param[opus, 0]
        u_d = param[opus, 1]
        h = param[opus, 2]
        lp = "True" if param[opus, 3]==1 else "False"
        d_detec = param[opus, 4]
        dcpa = param[opus, 5]
        group = param[opus, 6]
        u_d_asv = param[opus, 7]

        size = 8.0
        type = 'none'
        initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
        waypoints_asv = [[0.,0.],
                         [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]

        if opus == 0:
            input_file.write(f'OPUS    CLASS    U_D_ASV    LOC_PLAN    HEADING    U_D    DCPA    SIZE    PRIOR    D_DETEC    GROUP\n')
        input_file.write(f'{opus+1}    {class_scen}   {u_d_asv}    {lp}    {h}    {u_d}    {dcpa}    {size}    {type}    {d_detec}    {group}\n')

        # Creation of the launch files
        # cli_args0 = ['open_seas_bench', 'main_launch2.launch',
        #              f'trigger_shutdown:=1',
        #              f'initial_state:={initial_state_asv}',
        #              f'waypoints:={waypoints_asv}',
        #              f'u_d:={u_d_asv}',
        #              f'use_vo:={lp}',
        #              f'rviz:=False',
        #              f'opus:={opus+1}',
        #              f'output_file:=$(find open_seas_bench)/output/{serial}.txt']
        # roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
        # roslaunch_args0 = cli_args0[2:]
        #
        # cli_args1 = ['asv_obstacle_tracker', 'obst_simplified.launch',
        #             f'prior:=[{type}]',
        #             f'size:=[{size}]',
        #             f'heading:=[{h}]',
        #             f'u_d:=[{u_d}]',
        #             f't_collision:=[{t_collision}]',
        #             f'd_detection:=[{d_detec}]',
        #             f'dcpa:=[{dcpa}]',
        #             f'initial_state_asv:={initial_state_asv}']
        # roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
        # roslaunch_args1 = cli_args1[2:]
        # launch_files = [(roslaunch_file0, roslaunch_args0), (roslaunch_file1, roslaunch_args1)]
        #
        # launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        # launch.start()
        # launch.spin()




if __name__ == "__main__":

    rospack = rospkg.RosPack()
    root = tk.Tk()
    root.wm_withdraw()
    param = askopenfilename(title="Load a file :",filetypes=[('excel files','.xlsx', '.xls'),('all files','.*')],
                                          initialdir=f"{rospack.get_path('open_seas_bench')}/config/param/")
    root.destroy()

    if param:
        # Output parameters
        now = datetime.datetime.now()
        serial = now.strftime("%Y%m%d%H%M%S")[2:]

        # Write Input
        input = f"{rospack.get_path('open_seas_bench')}/input/{serial}.txt"
        f = open(input,'a')

        try :
            run(serial, f, param)
        except:
            f.close()
    else:
        print("No file chosen")
