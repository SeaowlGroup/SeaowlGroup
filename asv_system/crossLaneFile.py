#!/usr/bin/env python3

import roslaunch
import numpy as np
import datetime
import rospkg
import yaml

NB_PROCESS = 1

def run(serial, params, uuid) :

    rospack = rospkg.RosPack()
    first_opus = params[0][0]

    cli_args0 = ['asv_system', 'reaper.launch',
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
        nOb = scenar[3]
        rlw = scenar[4]*1852. #natical miles to m
        llw = scenar[5]*1852. #natical miles to m
        ld = scenar[6]*1852. #natical miles to m
        gp = 0

        # ASV parameters
        dAsv = 100. #initial and final distance to lanes
        initial_state_asv = [(ld/2+llw+dAsv)/np.tan(angle)/2,-ld/2-llw-dAsv,-angle, u_d,0.,0.]
        #Trajectory
        waypoints_asv = [[(ld/2+llw+dAsv)/np.tan(angle)/2,-ld/2-llw-dAsv],
                         [(ld/2+rlw+dAsv)/np.tan(angle)/2,ld/2+rlw+dAsv]]

        input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
        f = open(input,'a')
        f.write(f'{opus}    {angle}    {u_d}    {nOb}    {rlw}    {llw}    {ld}   {gp}\n')
        f.close()

        # Creation of the launch files
        cli_args1 = ['asv_system', 'crossLane.launch',
                    f'nOb:={nOb}',
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
                     f'output_file:=$(find asv_system)/output/{serial}.txt',
                     f'pos_end_waypoint:={waypoints_asv[-1]}']
        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
        roslaunch_args1 = cli_args1[2:]
        launch_files.append((roslaunch_file1, roslaunch_args1))
        #print([opus, angle, u_d, nOb, rlw, llw, ld])
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    launch.spin()


if __name__ == "__main__":

    yaml_file = open("config/param/crossLane.yaml", 'r')
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
    f.write(f'OPUS    ANGLE    U_D    N_OB    RLW    LLW    LD   GROUP\n')
    f.close()

    params = []
    opus = 1
    try:
        for angle in yaml_content['angle']:
            for u_d in yaml_content['u_d']:
                for nOb in yaml_content['nOb']:
                    for rlw in yaml_content['rlw']:
                        for llw in yaml_content['llw']:
                            for ld in yaml_content['ld']:
                                params.append([opus, angle, u_d, nOb, rlw, llw, ld])
                                opus += 1
                                if len(params) == NB_PROCESS:
                                    #try:
                                    run(serial, params, uuid)
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
