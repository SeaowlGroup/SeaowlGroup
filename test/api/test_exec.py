#!/usr/bin/env python3

import roslaunch
#import rospy
N = 1

def run(op,uuid):
    #cli_args = ['test','test.launch',f'opus:={op}']
    #roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    #roslaunch_args0 = cli_args[2:]
    #launch_files = [(roslaunch_file0, roslaunch_args0)]
    config = roslaunch.ROSLaunchConfig()
    opus = roslaunch.core.Param("opus",op)
    config.add_param(opus)
    node = roslaunch.core.Node("test","test_node.py","test_node",f'{op}',respawn=False, required=True)
    config.add_node(node)
    launch = roslaunch.ROSLaunchRunner(uuid, config)
    launch.launch()
    print("launched")
    launch.spin()

    #rospy.sleep(2)
    #launch.shutdown()


if __name__ == "__main__":
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    try:
        for k in range(N):
            run(k, uuid)
    except KeyboardInterrupt:
        print("done")
