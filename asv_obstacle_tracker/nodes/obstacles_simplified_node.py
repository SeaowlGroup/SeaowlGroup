#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray
from std_msgs.msg import Empty

class Obstacles(object):

    def __init__(self):
        self.prior = rospy.get_param("~prior", [])
        self.size = rospy.get_param("~size", [])
        self.heading = rospy.get_param("~heading", [])
        self.u_d = rospy.get_param("~u_d", [])
        self.t_collision = rospy.get_param("~t_collision", [])
        self.d_detection = rospy.get_param("~d_detection", [])
        self.dcpa = rospy.get_param("~dcpa", [])
        self.op = rospy.get_param("opusob",-1)
        print(f'this opus: {self.op}')

        self.initial_state_asv = rospy.get_param("~initial_state_asv")

        self.dt = rospy.get_param("~update_rate", .1)

        self.start = False

        self.asv_state = []

        self.br = tf.TransformBroadcaster()

        self.states_pub = rospy.Publisher("obstacle_states", StateArray, queue_size=1)


        self.start_subscriber = rospy.Subscriber("start_simulation", Empty,
                                                self.start_callback,
                                                queue_size=10)

        self._odom_subscriber = rospy.Subscriber("state", Odometry,
                                                    self._odom_callback,
                                                    queue_size=1)


    def run(self):
        statearray = StateArray()


        x_asv = self.initial_state_asv[0]
        y_asv = self.initial_state_asv[1]
        psi_asv = self.initial_state_asv[2]
        u_d_asv = self.initial_state_asv[3]

        N = len(self.u_d)
        calc_heading = (90-np.array(self.heading))*np.pi/180
        delay_time = np.zeros(N)
        for i in range(N) :

            psi = calc_heading[i]
            theta = psi_asv - psi
            t_col = self.t_collision[i]
            u = self.u_d[i]
            cpa = np.array([-u*np.sin(psi)+u_d_asv*np.sin(psi_asv),u*np.cos(psi)-u_d_asv*np.cos(psi_asv)])
            cpa = self.dcpa[i]*cpa/np.linalg.norm(cpa) #postion du cpa de l'obst par rapport à l'asv

            # Delay time
            if dist_init(u, u_d_asv, theta, t_col) < self.d_detection[i] :
                delay_time[i] = delay(u, u_d_asv, theta, t_col, self.d_detection[i])
                #delay_time[i] = 0
            # # Angle of the relative speed
            # c = np.sqrt(u**2 + u_d_asv**2 - 2*u*u_d_asv*np.cos(theta))
            # aux_angle = np.arccos((u_d_asv**2 + c**2 - u**2)/(2*u_d_asv*c))
            # alpha = 90+aux_angle
            # # Trajectory
            # print(self.dcpa[i]*np.sin(alpha), -self.dcpa[i]*np.cos(alpha))
            # x = (t_col*u_d_asv*np.cos(psi_asv) - (t_col-delay_time[i])*u*np.cos(psi) +
            #     x_asv + self.dcpa[i]*np.sin(alpha))
            # y = (t_col*u_d_asv*np.sin(psi_asv) - (t_col-delay_time[i])*u*np.sin(psi) +
            #     y_asv - self.dcpa[i]*np.cos(alpha))
            # Trajectory
            #coef = 1/np.sqrt((1+4*np.sqrt(2))/(4+4*np.sqrt(2)))
            #coef = 1/np.sqrt((1+np.sin(psi))/2)
            #print("coef : ", coef)
            x = (x_asv
                +t_col*u_d_asv*np.cos(psi_asv)
                +cpa[0]
                -(t_col-delay_time[i])*u*np.cos(psi))
            y = (y_asv
                +t_col*u_d_asv*np.sin(psi_asv)
                +cpa[1]
                -(t_col-delay_time[i])*u*np.sin(psi))


            statearray.states.append(State())
            statearray.states[i].header.id = i
            statearray.states[i].header.name = "Ship " + str(i)
            statearray.states[i].header.radius = self.size[i]
            statearray.states[i].header.prior = self.prior[i]

            statearray.states[i].x = x
            statearray.states[i].y = y
            statearray.states[i].psi = psi
            statearray.states[i].u = u
            statearray.states[i].v = 0.0
            statearray.states[i].r = 0.0

        r = rospy.Rate(1/self.dt)


        while (not rospy.is_shutdown()):

            for i in range(N) :
                if (len(self.asv_state)>0 and statearray.states[i].x-self.asv_state[0])**2 + (statearray.states[i].y-self.asv_state[1])**2 < (self.d_detection[i]+self.size[i]/2)**2 :
                    self.states_pub.publish(statearray) # NE MARCHE QUE SI UN SEUL OBSTACLE

                self.br.sendTransform((statearray.states[i].x,statearray.states[i].y,0),
                                tf.transformations.quaternion_from_euler(0,0,statearray.states[i].psi),
                                rospy.Time.now(),
                                str(self.op)+statearray.states[i].header.name,
                                "map")

                statearray.states[i].x += self.u_d[i]*np.cos(calc_heading[i])*self.dt
                statearray.states[i].y += self.u_d[i]*np.sin(calc_heading[i])*self.dt

            r.sleep()

    def start_callback(self, data):
        self.start = True

    def _odom_callback(self, data):
        if len(self.asv_state) ==0:
            self.asv_state = np.zeros(2)
        self.asv_state[0] = data.pose.pose.position.x
        self.asv_state[1] = data.pose.pose.position.y


# Utils
def dist_init(u_d, u_d_asv, theta, t_col) :
    return (u_d_asv*t_col)**2 + (u_d*t_col)**2 - 2*u_d*u_d_asv*(t_col**2)*np.cos(theta)

def delay(u_d, u_d_asv, theta, t_col, d_detec) :
    return t_col - d_detec/np.sqrt(u_d**2 + u_d_asv**2 - 2*u_d_asv*u_d*np.cos(theta))


if __name__ == "__main__":
    rospy.init_node("obstacles_controller")

    obst = Obstacles()
    r = rospy.Rate(5.)
    while not obst.start :
        r.sleep()
    obst.run()
