#!/usr/bin/env python3

import roslaunch
#import rospy
N = 100

def run(op,uuid):
    cli_args = ['test','test.launch',f'opus:={op}']
    roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args0 = cli_args[2:]
    launch_files = [(roslaunch_file0, roslaunch_args0)]

    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    launch.spin_once()
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
