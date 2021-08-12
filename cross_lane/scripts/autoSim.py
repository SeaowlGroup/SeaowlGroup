#!/usr/bin/env python3

import roslaunch
import numpy as np
import datetime
import rospkg
import yaml,os,sys

NB_PROCESS = 4
OPUS_START = 1
SERIAL_TO_UPDATE = ''

def run(serial, params, uuid) :

    rospack = rospkg.RosPack()
    first_opus = params[0][0]

    cli_args0 = ['asv_referee', 'reaper.launch',
                 f'nb_processes:={NB_PROCESS}',
                 f'opus:={first_opus}']
    roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]
    roslaunch_args0 = cli_args0[2:]
    launch_files = [(roslaunch_file0, roslaunch_args0)]

    lp = True
    size = 8.0
    type = None

    for scenar in params:

        opus = scenar[0]
        angle = scenar[1]/180*np.pi #degrees ro radians
        u_d = scenar[2]*0.514444    #knots to m/s
        rld = scenar[3]/10000       #1/hm² to 1/m²
        lld = scenar[4]/10000       #1/hm² to 1/m²
        rlw = scenar[5]*1852        #nm to m
        llw = scenar[6]*1852        #nm to m
        ld = scenar[7]*1852         #nm to m
        gp = 0

        # ASV parameters
        dAsv = 100. #initial and final distance to lanes
        initial_state_asv = [-(ld/2+llw+dAsv)/np.tan(angle),-(ld/2+llw+dAsv),angle, u_d,0.,0.]
        #Trajectory
        waypoints_asv = [[-(ld/2+llw+dAsv)/np.tan(angle),-(ld/2+llw+dAsv)],
                         [(ld/2+rlw+dAsv)/np.tan(angle),ld/2+rlw+dAsv]]

        input = f"{rospack.get_path('cross_lane')}/input/{serial}.txt"
        f = open(input,'a')
        f.write(f'{opus}    {angle}    {u_d}    {rld}    {lld}    {rlw}    {llw}    {ld}   {gp}\n')
        f.close()

        # Creation of the launch files
        cli_args1 = ['cross_lane', 'crossLane.launch',
                     f'rld:={rld}',
                     f'lld:={lld}',
                     f'rlw:={rlw}',
                     f'llw:={llw}',
                     f'ld:={ld}',
                     f'trigger_shutdown:=0',
                     f'initial_state:={initial_state_asv}',
                     f'waypoints:={waypoints_asv}',
                     f'u_d:={u_d}',
                     f'use_vo:={True}',
                     f'rviz:=False',
                     f'opus:={opus}',
                     f'output_file:=$(find cross_lane)/output/{serial}.txt',
                     f'pos_end_waypoint:={waypoints_asv[-1]}']
        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
        roslaunch_args1 = cli_args1[2:]
        launch_files.append((roslaunch_file1, roslaunch_args1))
        #print([opus, angle, u_d, nOb, rlw, llw, ld])
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    launch.spin()


if __name__ == "__main__":

    if len(sys.argv) <= 3 :
        print(f'Usage: {sys.argv[0]} <bench> <op_start> <op_end> <serial>')
        sys.exit(1)

    op_start =  int(sys.argv[2])
    op_end = int(sys.argv[3])

    if len(sys.argv) <= 4 :
        now = datetime.datetime.now()
        serial = now.strftime("%Y%m%d%H%M%S")[2:]
    else:
        serial = sys.argv[4]

    rospack = rospkg.RosPack()

    yaml_file = open(f"{rospack.get_path('cross_lane')}/config/param/crossLane.yaml", 'r')
    yaml_content = yaml.safe_load(yaml_file)

    # UUID
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # Output parameters
    if len(SERIAL_TO_UPDATE) == 0: 
        now = datetime.datetime.now()
        serial = now.strftime("%Y%m%d%H%M%S")[2:]
    else:
        serial = SERIAL_TO_UPDATE

    # Write Input
    input = f"{rospack.get_path('cross_lane')}/input/{serial}.txt"
    f = open(input,'a')
    f.write(f'OPUS    ANGLE    U_D    RLD    LLD    RLW    LLW    LD   GROUP\n')
    f.close()

    opus = 1
    params = []
    try:
        for angle in yaml_content['angle']:
            for u_d in yaml_content['u_d']:
                for rld in yaml_content['rld']:
                    for lld in yaml_content['rld']:
                        for rlw in yaml_content['rlw']:
                            for llw in yaml_content['llw']:
                                for ld in yaml_content['ld']:
                                    if opus > op_end:
                                        sys.exit(0)
                                    if opus >= op_start:
                                        params.append([opus, angle, u_d, rld, lld, rlw, llw, ld])
                                    opus += 1

                                    if len(params) == NB_PROCESS or  (len(params) > 0 and opus > op_end):
                                        if os.path.exists('nohup.out'):
                                            os.remove('nohup.out')
                                        if os.path.exists('nohup.err'):
                                            os.remove('nohup.err')
                                        run(serial, params, uuid)
                                        params = []
    except KeyboardInterrupt:
        pass
