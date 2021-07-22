#!/usr/bin/env python3

import roslaunch
import numpy as np
import time
import datetime
import rospkg
from multiprocessing import Process

def run(serial, input_file, params, uuid) :

    # ASV parameters
    calc_heading_asv = np.pi/2
    u_d_asv = 5.0
    initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
    t_sim = 75
    t_collision = 45
    #Trajectory
    waypoints_asv = [[0.,0.],
                     [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]

    launch_files = []

    for opus in range(len(params)):
        param = params[opus]

        class_scen = param[0]
        u_d = param[1]
        h = param[2]
        lp = param[3]
        d_detec = param[4]
        dcpa = param[5]
        group = param[6]
        size = param[7]
        type = param[8]

        # param = [class_scen, u_d, h, lp, d_detec, dcpa, group, size, type]
        input_file.write(f'{opus+1}    {class_scen}   {u_d_asv}    {lp}    {h}    {u_d}    {dcpa}    {size}    {type}    {d_detec}    {group}\n')

        # Creation of the launch files
        cli_args0 = ['asv_system', 'main_launch3.launch',
                     f'trigger_shutdown:=0',
                     f'initial_state:={initial_state_asv}',
                     f'waypoints:={waypoints_asv}',
                     f'u_d:={u_d_asv}',
                     f'use_vo:={lp}',
                     f'rviz:=False',
                     f'opus:={opus+1}',
                     f'output_file:=$(find asv_system)/output/{serial}.txt']
        roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
        roslaunch_args0 = cli_args0[2:]
        launch_files.append((roslaunch_file0, roslaunch_args0))

        cli_args1 = ['asv_obstacle_tracker', 'obst_simplified2.launch',
                    f'prior:=[{type}]',
                    f'size:=[{size}]',
                    f'heading:=[{h}]',
                    f'u_d:=[{u_d}]',
                    f't_collision:=[{t_collision}]',
                    f'd_detection:=[{d_detec}]',
                    f'dcpa:=[{dcpa}]',
                    f'initial_state_asv:={initial_state_asv}',
                    f'opus:={opus+1}']
        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
        roslaunch_args1 = cli_args1[2:]
        launch_files.append((roslaunch_file1, roslaunch_args1))

    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    launch.spin()


if __name__ == "__main__":

    # UUID
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # # ASV parameters
    # calc_heading_asv = np.pi/2
    # u_d_asv = 5.0
    # initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
    # t_sim = 75
    # t_collision = 45
    # #Trajectory
    # waypoints_asv = [[0.,0.],
    #                  [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]


    class_scen = ['CF', 'CL']
    u_d = [5.0, 6.0]
    h = [180, 225]
    lp = ["True", "True"]
    d_detec = [500, 500]
    dcpa = [0., 10.]
    group = [1, 1]
    size = [8.0, 8.0]
    type = ['none', 'none']

    # param = [class_scen, u_d, h, lp, d_detec, dcpa, group, size, type]
    params = [['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none'],
              ['CF', 10.0, 180, "True", 50, 0., 1, 8.0, 'none']]

    # Output parameters
    now = datetime.datetime.now()
    serial = now.strftime("%Y%m%d%H%M%S")[2:]

    rospack = rospkg.RosPack()
    input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
    f = open(input,'a')
    f.write(f'OPUS    CLASS    U_D_ASV    LOC_PLAN    HEADING    U_D    DCPA    SIZE    PRIOR    D_DETEC    GROUP\n')

    run(serial, f, params, uuid)


    f.close()
