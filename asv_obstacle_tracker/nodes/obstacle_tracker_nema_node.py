phiphiphi#!/usr/bin/env python3
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
    # resolution =
    # origin_x =
    # origin_y =
    #
    # if port < 0 :
    #     rospy.signal_shutdown("No host address")
    host = "239.192.0.1"
    port = 60001

    resolution = 1.0
    origin_lat = 0.0
    origin_lon = 0.0

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

        dLat = decoded_msg.content['lat'] - origin_lat
        dLon = decoded_msg.content['lon'] - origin_lon
        phi = np.pi*origin_lat/180
        statearray.states[num].x = dLon*(111132.954*np.cos(phi) - 93.5*np.cos(3*phi) + 0.118*np.cos(5*phi))
        statearray.states[num].y = dLat*(111132.954 - 559.822*np.cos(2*phi) + 1.175*np.cos(4*phi) - 0.0023*np.cos(4*phi))
        statearray.states[num].psi = decoded_msg.content['heading']*np.pi/180
        statearray.states[num].u = decoded_msg.content['speed']
        statearray.states[num].v = 0.0
        statearray.states[num].r = decoded_msg.content['turn']/6

        pub.publish(statearray)

    rospy.spin()
