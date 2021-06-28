#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from asv_msgs.msg import State, StateArray


def dist_init(u_d, u_d_asv, theta, t_col) :
    return (u_d_asv*t_col)**2 + (u_d*t_col)**2 - 2*u_d*u_d_asv*(t_col**2)*np.cos(theta)

def delay(u_d, u_d_asv, theta, t_col, d_detec) :
    return t_col - d_detec/np.sqrt(u_d**2 + u_d_asv**2 - 2*u_d_asv*u_d*np.cos(theta))

if __name__ == "__main__":
    rospy.init_node("obstacles_controller")

    prior = rospy.get_param("~prior", [])
    size = rospy.get_param("~size", [])
    heading = rospy.get_param("~heading", [])
    u_d = rospy.get_param("~u_d", [])
    t_collision = rospy.get_param("~t_collision", [])
    d_detection = rospy.get_param("~d_detection", [])
    dcpa = rospy.get_param("~dcpa", [])

    initial_state_asv = rospy.get_param("~initial_state_asv")

    dt = rospy.get_param("~update_rate", .1)

    states_pub = rospy.Publisher("/obstacle_states", StateArray, queue_size=1)
    statearray = StateArray()

    x_asv = initial_state_asv[0]
    y_asv = initial_state_asv[1]
    psi_asv = initial_state_asv[2]
    u_d_asv = initial_state_asv[3]

    N = len(u_d)
    calc_heading = (90-np.array(heading))*np.pi/180
    delay_time = np.zeros(N)
    for i in range(N) :

        psi = calc_heading[i]
        theta = psi_asv - psi
        t_col = t_collision[i]
        u = u_d[i]

        # Delay time
        if dist_init(u, u_d_asv, theta, t_col) < d_detection[i] :
            delay_time[i] = delay(u, u_d_asv, theta, t_col, d_detection[i])
            #delay_time[i] = 0
        # Trajectory
        x = (t_col*u_d_asv*np.cos(psi_asv) - (t_col-delay_time[i])*u*np.cos(psi) +
             x_asv + dcpa[i]*np.cos(psi_asv))
        y = (t_col*u_d_asv*np.sin(psi_asv) - (t_col-delay_time[i])*u*np.sin(psi) +
             y_asv + dcpa[i]*np.sin(psi_asv))


        statearray.states.append(State())
        statearray.states[i].header.id = i
        statearray.states[i].header.name = "Ship " + str(i)
        statearray.states[i].header.radius = size[i]
        statearray.states[i].header.prior = prior[i]

        statearray.states[i].x = x
        statearray.states[i].y = y
        statearray.states[i].psi = psi
        statearray.states[i].u = u
        statearray.states[i].v = 0.0
        statearray.states[i].r = 0.0


    r = rospy.Rate(1/dt)

    th_state_asv = np.array([x_asv, y_asv, psi_asv, u_d_asv])

    while not rospy.is_shutdown():


        for i in range(N) :
            if (statearray.states[i].x-th_state_asv[0])**2 + (statearray.states[i].y-th_state_asv[1])**2 < (d_detection[i]+size[i]/2)**2 :
                states_pub.publish(statearray) # NE MARCHE QUE SI UN SEUL OBSTACLE
                br = tf.TransformBroadcaster()
                br.sendTransform((statearray.states[i].x,statearray.states[i].y,0),
                                tf.transformations.quaternion_from_euler(0,0,statearray.states[i].psi),
                                rospy.Time.now(),
                                statearray.states[i].header.name,
                                "map")

            statearray.states[i].x += u_d[i]*np.cos(calc_heading[i])*dt
            statearray.states[i].y += u_d[i]*np.sin(calc_heading[i])*dt
            th_state_asv[0] += th_state_asv[3]*np.cos(th_state_asv[2])*dt
            th_state_asv[1] += th_state_asv[3]*np.sin(th_state_asv[2])*dt

        r.sleep()
