#!/usr/bin/env python3

import roslaunch
import yaml
import numpy as np

yaml_file = open("config/param2.yaml", 'r')
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

# Main launch file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

asv = yaml_content['asv']
u_d_asv = asv['u_d']
true_heading_asv = asv['heading']
calc_heading_asv = (90-true_heading_asv)*np.pi/180
initial_state_asv = [0.,0.,calc_heading_asv, u_d_asv,0.,0.]
t_sim = asv['t_sim']
waypoints_asv = [[0.,0.], [t_sim*u_d_asv*np.cos(calc_heading_asv), t_sim*u_d_asv*np.sin(calc_heading_asv)]]

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
             f'there_are_waves:={there_are_waves}']
roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
roslaunch_args0 = cli_args0[2:]

# Obstacle tracker
obstacles = yaml_content['obstacles']
N = len(obstacles)

if N == 0 :
    launch_files = [(roslaunch_file0, roslaunch_args0)]
else :
    cli_args1 = ['asv_obstacle_tracker', 'default.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
    launch_files = [(roslaunch_file0, roslaunch_args0), roslaunch_file1]

# Obstacles
for i in range(N) :
    ship = obstacles[f'ship{i+1}']
    type = ship['type']
    size = ship['size']
    true_heading = ship['heading']
    u_d = ship['u_d']
    t_collision = ship['t_collision']

    calc_heading = (90-true_heading)*np.pi/180

    or_x = 0.0
    or_y = 0.0
    #u_d_asv = 5.0
    #heading_asv = 1.56
    first_point_x = t_collision*(u_d_asv*np.cos(calc_heading_asv) - u_d*np.cos(calc_heading)) + or_x
    first_point_y = t_collision*(u_d_asv*np.sin(calc_heading_asv) - u_d*np.sin(calc_heading)) + or_y
    last_point_x = first_point_x + 2*t_collision*u_d*np.cos(calc_heading)
    last_point_y = first_point_y + 2*t_collision*u_d*np.sin(calc_heading)

    initial_state = [first_point_x, first_point_y, calc_heading, u_d, 0., 0.]
    waypoints = [[first_point_x, first_point_y], [last_point_x, last_point_y]]
    shipname = type+f'{i+1}'

    cli_args = ['asv_system', 'obstacles2.launch',
                f'initial_state:={initial_state}',
                f'waypoints:={waypoints}',
                f'shipname:={shipname}',
                f'u_d:={u_d}',
                f'size:={size}',
                f'Fx_current:={Fx_current}',
                f'Fy_current:={Fy_current}',
                f'there_are_waves:={there_are_waves}']

    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[2:]

    launch_files.append((roslaunch_file, roslaunch_args))

launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
launch.start()

launch.spin()
