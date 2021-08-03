#!/usr/bin/env python3

import roslaunch
import rospy
#import rospkg
N = 10

def run(op,uuid):
    cli_args = ['test','test.launch',f'opus:={op}']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    print(roslaunch_file)
    roslaunch_args = cli_args[2:]
    launch_files = [(roslaunch_file, roslaunch_args)]
    #config = roslaunch.ROSLaunchConfig()
    #opus = roslaunch.core.Param("opus",op)
    #config.add_param(opus)
    #node = roslaunch.core.Node("test","test_node.py","test_node",f'{op}',respawn=False, required=True)
    #config.add_node(node)
    #launch = roslaunch.ROSLaunchRunner(uuid, config)
    #launch = roslaunch.scriptapi.ROSLaunch()
    #launch.start()
    #print("launched")
    #launch.spin()

    #rospy.sleep(2)
    #launch.shutdown()
    
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    launch.spin()
    


if __name__ == "__main__":
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    try:
        for k in range(N):
            run(k,uuid)
            print("sleep")
            rospy.sleep(1)
            print("wake")
    except KeyboardInterrupt:
        print("done")
