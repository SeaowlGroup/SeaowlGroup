#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
import os
import sys


def callback(data):
    global count
    count -= 1


if __name__ == "__main__" :

    rospy.init_node("Reaper")

    dt = 1/rospy.get_param("~update_rate", 5.)
    N = rospy.get_param("nb_processes", 1)
    opus = rospy.get_param("opus", 0)
    # N = int(sys.argv[1])

    suscribers = []
    count = N

    for i in range(N):
        suscribers.append(rospy.Subscriber(f"/{opus+i}/end_simulation", Empty,
                                           callback,
                                           queue_size=20))


    r = rospy.Rate(dt)
    while (not rospy.is_shutdown()):
        if count == 0:
            # nodes = os.popen("rosnode list").readlines()
            # for i in range(len(nodes)):
            #     nodes[i] = nodes[i].replace("\n","")
            #
            # for node in nodes:
            #     os.system("rosnode kill "+ node)
            rospy.signal_shutdown("Set of scenarios ended")

        r.sleep()
