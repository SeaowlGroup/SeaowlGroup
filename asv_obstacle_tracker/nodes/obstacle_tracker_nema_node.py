#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray

import numpy as np
import matplotlib.pyplot as plt
from pyais.stream import UDPStream

if __name__ == "__main__":

    rospy.init_node("obstacle_tracker_node")
    print("Obstacle tracker launched")

    # host = rospy.get_param("host", "")
    # port = rospy.get_param("port", -1)
    #
    # if port < 0 :
    #     rospy.signal_shutdown("No host address")
    host = "239.192.0.1"
    port = 60001

    statearray = StateArray()
    id = dict()
    pub = rospy.Publisher("/obstacle_states", StateArray, queue_size=1)

    count = 0

    for msg in UDPStream(host, port):
        decoded_msg = msg.decode()
        mmsi = decoded_msg.content['mmsi']
        if mmsi in id :
            num = id[mmsi]

        else :
            count +=1
            num = count

            statearray.states[num].header.id = num
            statearray.states[num].header.name = "Ship " + str(mmsi)
            statearray.states[num].header.radius = 8.0 # temporary

        statearray.states[num].x = decoded_msg.content['lon']
        statearray.states[num].y = decoded_msg.content['lat']
        statearray.states[num].psi = decoded_msg.content['heading']*360/6.28318530718
        statearray.states[num].u = decoded_msg.content['speed']
        #statearray.states[num].v = data.twist.twist.linear.y  --> ???
        statearray.states[num].r = decoded_msg.content['turn']

        pub.publish(statearray)

    rospy.spin()
