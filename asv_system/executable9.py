#!/usr/bin/env python3

import roslaunch
import numpy as np
import time
import datetime
import rospkg
import yaml
import sys

NB_PROCESS = 6

def run(serial, input_file, params, uuid) :

    cli_args0 = ['asv_system', 'reaper.launch',
                 f'nb_processes:={NB_PROCESS}']
    roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
    roslaunch_args0 = cli_args0[2:]
    launch_files = [(roslaunch_file0, roslaunch_args0)]

    t_sim = 75
    lp = True
    size = 8.0
    type = None

    for scenar in params:

        h = scenar[0]
        u_d = scenar[1]
        u_d_asv = scenar[2]
        dcpa = scenar[3]
        d_detec = scenar[4]
        opus = scenar[5]

        if (np.abs(h)<=20):
            t_collision = 15
            class_scen = 'OVERTAKING'
            group = 1
        elif (h>20 and h<150):
            t_collision = 45
            class_scen = 'CROSSING_LEFT'
            group = 2
        elif (h>=150 and h<=210):
            t_collision = 45
            class_scen = 'HEAD_ON'
            group = 3
        else:
            class_scen = 'CROSSING_RIGHT'
            group = 4

        # ASV parameters
        calc_heading_asv = np.pi/2
        initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
        #Trajectory
        waypoints_asv = [[0.,0.],
                         [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]

        input_file.write(f'{opus}    {class_scen}   {u_d_asv}    {lp}    {h}    {u_d}    {dcpa}    {size}    {type}    {d_detec}    {group}\n')

        # Creation of the launch files
        cli_args1 = ['asv_system', 'main_launch3.launch',
                     f'trigger_shutdown:=0',
                     f'initial_state:={initial_state_asv}',
                     f'waypoints:={waypoints_asv}',
                     f'u_d:={u_d_asv}',
                     f'use_vo:={lp}',
                     f'rviz:=False',
                     f'opus:={opus}',
                     f'output_file:=$(find asv_system)/output/{serial}.txt']
        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
        roslaunch_args1 = cli_args1[2:]
        launch_files.append((roslaunch_file1, roslaunch_args1))

        cli_args2 = ['asv_obstacle_tracker', 'obst_simplified2.launch',
                    f'prior:=[{type}]',
                    f'size:=[{size}]',
                    f'heading:=[{h}]',
                    f'u_d:=[{u_d}]',
                    f't_collision:=[{t_collision}]',
                    f'd_detection:=[{d_detec}]',
                    f'dcpa:=[{dcpa}]',
                    f'initial_state_asv:={initial_state_asv}',
                    f'opus:={opus}']
        roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
        roslaunch_args2 = cli_args2[2:]
        launch_files.append((roslaunch_file2, roslaunch_args2))

    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    launch.spin()


if __name__ == "__main__":

    yaml_file = open("config/param/param4.yaml", 'r')
    yaml_content = yaml.safe_load(yaml_file)

    # UUID
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # Output parameters
    now = datetime.datetime.now()
    serial = now.strftime("%Y%m%d%H%M%S")[2:]

    # Write Input
    rospack = rospkg.RosPack()
    input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
    f = open(input,'a')

    params = []
    opus = 1
    try:
        for h in yaml_content['heading']:
            for u_d in yaml_content['u_d']:
                for u_d_asv in yaml_content['u_d_asv']:
                    for dcpa in yaml_content['dcpa']:
                        for d_detec in yaml_content['d_detection_jb']: ############################################
                            if (h<340 and h>20) or (np.abs(u_d-u_d_asv)>2.57):
                                params.append([h, u_d, u_d_asv, dcpa, d_detec, opus])
                                opus += 1
                                if len(params) == NB_PROCESS:
                                    #try:
                                    run(serial, f, params, uuid)
                                    params = []
                                    # except:
                                    #     print("Unexpected error:", sys.exc_info()[0])
                                    #     output = f"{rospack.get_path('asv_system')}/output/{serial}.txt"
                                    #     g = open(input,'a')
                                    #     g.write(f'{opus+1} nan nan nan nan nan nan nan nan nan nan nan nan nan\n')
                                    #     g.close()
                                    #     params = []
    except KeyboardInterrupt:
        pass

    f.close()
