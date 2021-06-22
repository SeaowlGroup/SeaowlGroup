#!/usr/bin/env python3

import roslaunch
import yaml

yaml_file = open("config/param1.yaml", 'r')
yaml_content = yaml.safe_load(yaml_file)

traj = yaml_content['obstacles']
if 'mapfile' in yaml_content :
    map = yaml_content['mapfile']
else :
    map = 'None'
u = yaml_content['use_sim_time']
t = yaml_content['trigger_shutdown']
c = yaml_content['coast_margin']
N = len(traj)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args0 = ['asv_system', 'mozambique_simplified.launch',
             f'use_sim_time:={u}',
             f'trigger_shutdown:={t}',
             f'coast_margin:={c}',
             f'mapfile:={map}']
roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
roslaunch_args0 = cli_args0[2:]

if N == 0 :
    launch_files = [(roslaunch_file0, roslaunch_args0)]
else :
    cli_args1 = ['asv_obstacle_tracker', 'default.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
    launch_files = [(roslaunch_file0, roslaunch_args0), roslaunch_file1]

for i in range(N) :
    # Start another node
    cli_args = ['asv_system', 'obstacles.launch',
                f'initial_state:={traj[i][0]}',
                f'waypoints:=[{traj[i][0][:2]}, {traj[i][1]}]',
                f'shipname:=ship{i+1}']

    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[2:]

    launch_files.append((roslaunch_file, roslaunch_args))

launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
launch.start()

launch.spin()
