#!/usr/bin/env python3

import roslaunch
import yaml
import numpy as np
import time
import datetime

def run() :
    yaml_file = open("config/param3.yaml", 'r')
    yaml_content = yaml.safe_load(yaml_file)

    if 'mapfile' in yaml_content :
        map = yaml_content['mapfile']
    else :
        map = 'None'
    use_sim_time = yaml_content['use_sim_time']
    trigger_shutdown = yaml_content['trigger_shutdown']
    coast_margin = yaml_content['coast_margin']
    angle_current = (270-yaml_content['direction_current'])*np.pi/180
    angle_wind = (270-yaml_content['direction_wind'])*np.pi/180
    Fx_current = yaml_content['F_current']*np.cos(angle_current) + yaml_content['F_wind']*np.cos(angle_wind)
    Fy_current = yaml_content['F_current']*np.sin(angle_current) + yaml_content['F_wind']*np.sin(angle_wind)
    there_are_waves = yaml_content['there_are_waves']

    # UUID
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # Output parameters
    now = datetime.datetime.now()
    serial = now.strftime("%Y%m%d%H%M%S")[2:]
    opus = 0
    # ASV parameters
    asv = yaml_content['asv']
    u_d_asv = asv['u_d']
    true_heading_asv = asv['heading']
    calc_heading_asv = (90-true_heading_asv)*np.pi/180
    initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
    t_sim = asv['t_sim']
    #Trajectory
    waypoints_asv = [[0.,0.],
                     [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]
    # Obstacles
    obstacles = yaml_content['obstacles']
    N = len(obstacles)
    # Preparation of the different scripts
    ship = obstacles['ship1']
    for size in ship['size'] :
        for h in ship['heading'] :
            for u_d in ship['u_d'] :
                for dcpa in ship['dcpa'] :
                    for type in ship['prior'] :
                        for d_detec in ship['d_detection'] :
                            print('=====================================')
                            print('size : ', size)
                            print('heading : ', h)
                            print('u_d : ', u_d)
                            print('dcpa : ', dcpa)
                            print('prior : ', type)
                            print('d_detec : ', d_detec)
                            print('=====================================')
                            # Obstacles
                            #for i in range(N) :
                            #ship = obstacles[f'ship{i+1}']
                            opus += 1
                            t_collision = ship['t_collision']
                            # Creation of the launch files
                            cli_args0 = ['asv_system', 'main_launch2.launch',
                                         f'use_sim_time:={use_sim_time}',
                                         f'trigger_shutdown:={trigger_shutdown}',
                                         f'coast_margin:={coast_margin}',
                                         f'mapfile:={map}',
                                         f'initial_state:={initial_state_asv}',
                                         f'waypoints:={waypoints_asv}',
                                         f'u_d:={u_d_asv}',
                                         f'Fx_current:={Fx_current}',
                                         f'Fy_current:={Fy_current}',
                                         f'there_are_waves:={there_are_waves}',
                                         f'use_vo:=True',
                                         f'opus:={opus}',
                                         f'output_file:=/home/adrien/catkin_ws/src/seaowl/asv_system/output/{serial}.txt']
                            roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
                            roslaunch_args0 = cli_args0[2:]

                            if N == 0 :
                                launch_files = [(roslaunch_file0, roslaunch_args0)]
                            else :
                                cli_args1 = ['asv_obstacle_tracker', 'obst_simplified.launch',
                                            f'prior:=[{type}]',
                                            f'size:=[{size}]',
                                            f'heading:=[{h}]',
                                            f'u_d:=[{u_d}]',
                                            f't_collision:=[{t_collision}]',
                                            f'd_detection:=[{d_detec}]',
                                            f'dcpa:=[{dcpa}]',
                                            f'initial_state_asv:={initial_state_asv}']
                                roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
                                roslaunch_args1 = cli_args1[2:]
                                launch_files = [(roslaunch_file0, roslaunch_args0), (roslaunch_file1, roslaunch_args1)]

                            # launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
                            # launch.start()
                            # time.sleep(10)
                            # cli_args = ['asv_system', 'obstacles2.launch',
                            #             f'initial_state:={initial_state}',
                            #             f'waypoints:={waypoints}',
                            #             f'shipname:={shipname}',
                            #             f'u_d:={u_d}',
                            #             f'size:={size}',
                            #             f'prior:={type}',
                            #             f'Fx_current:={Fx_current}',
                            #             f'Fy_current:={Fy_current}',
                            #             f'there_are_waves:={there_are_waves}',
                            #             f'node_start_delay:={0.0}'] #max(0., delay_time-3.0)
                            #
                            # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
                            # roslaunch_args = cli_args[2:]
                            #
                            # launch_files = [(roslaunch_file, roslaunch_args)]
                            launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
                            try :
                                launch.start()
                                launch.spin()
                            except roslaunch.core.RLException :
                                return
if __name__ == "__main__":
    run()
