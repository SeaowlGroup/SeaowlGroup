#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray
from std_msgs.msg import Empty

class Obstacles(object):

    def __init__(self):
        self.op = rospy.get_param("opus",1)
        self.nOb = int(rospy.get_param("~nOb", 3))
        self.prior = np.array(rospy.get_param("~prior", self.nOb*['none']))
        self.size = np.array(rospy.get_param("~size", self.nOb*[8.]))
        self.psi = (90-np.array(rospy.get_param("~heading", [k*360./self.nOb for k in range(self.nOb)])))*np.pi/180
        self.u_d = np.array(rospy.get_param("~u_d", self.nOb*[5.]))
        self.t_collision = np.array(rospy.get_param("~t_collision", self.nOb*[15.]))
        self.d_detection = np.array(rospy.get_param("~d_detection", self.nOb*[500.]))
        self.dcpa = np.array(rospy.get_param("~dcpa", self.nOb*[0.]))
        self.stateArray = StateArray()

        self.initial_state_asv = rospy.get_param("~initial_state_asv",[0.,0.,0.,0.])

        self.dt = rospy.get_param("~update_rate", .1)

        self.start = False

        self.asvPose = []

        self.br = tf.TransformBroadcaster()

        self.states_pub = rospy.Publisher("obstacle_states", StateArray, queue_size=1)

        self.obStatesRef_pub = rospy.Publisher("obStatesRef", StateArray, queue_size=1)

        self.start_subscriber = rospy.Subscriber("start_simulation", Empty,
                                                self.start_callback,
                                                queue_size=10)

        self._odom_subscriber = rospy.Subscriber("state", Odometry,
                                                    self._odom_callback,
                                                    queue_size=1)

    def initState(self):
        x_asv = self.initial_state_asv[0]
        y_asv = self.initial_state_asv[1]
        psi_asv = self.initial_state_asv[2]
        u_d_asv = self.initial_state_asv[3]

        delay_time = np.zeros(self.nOb)

        for i in range(self.nOb) :

            psi = self.psi[i]
            theta = psi_asv - psi
            t_col = self.t_collision[i]
            u = self.u_d[i]
            cpa = np.array([-u*np.sin(psi)+u_d_asv*np.sin(psi_asv),u*np.cos(psi)-u_d_asv*np.cos(psi_asv)])
            cpa = self.dcpa[i]*cpa/np.linalg.norm(cpa) #postion du cpa de l'obst par rapport à l'asv

            # Delay time
            if dist_init(u, u_d_asv, theta, t_col) < self.d_detection[i] :
                delay_time[i] = delay(u, u_d_asv, theta, t_col, self.d_detection[i])
      
            x = (x_asv
                +t_col*u_d_asv*np.cos(psi_asv)
                +cpa[0]
                -(t_col-delay_time[i])*u*np.cos(psi))
            y = (y_asv
                +t_col*u_d_asv*np.sin(psi_asv)
                +cpa[1]
                -(t_col-delay_time[i])*u*np.sin(psi))


            self.stateArray.states.append(State())
            self.stateArray.states[i].header.id = i
            self.stateArray.states[i].header.name = "Ship " + str(i)
            self.stateArray.states[i].header.radius = self.size[i]
            self.stateArray.states[i].header.prior = self.prior[i]

            self.stateArray.states[i].x = x
            self.stateArray.states[i].y = y
            self.stateArray.states[i].psi = psi
            self.stateArray.states[i].u = u
            self.stateArray.states[i].v = 0.0
            self.stateArray.states[i].r = 0.0

    def update(self):
        for i in range(self.nOb):
            self.stateArray.states[i].x += self.u_d[i]*np.cos(self.psi[i])*self.dt
            self.stateArray.states[i].y += self.u_d[i]*np.sin(self.psi[i])*self.dt
    
    def publish(self):
        self.obStatesRef_pub.publish(self.stateArray)
        obStates = StateArray()
        for i in range(self.nOb) :
                self.br.sendTransform((self.stateArray.states[i].x,self.stateArray.states[i].y,0),
                                    tf.transformations.quaternion_from_euler(0,0,self.stateArray.states[i].psi),
                                    rospy.Time.now(),
                                    str(self.op)+self.stateArray.states[i].header.name,
                                    "map")
                if (len(self.asvPose) == 2 and (self.stateArray.states[i].x-self.asvPose[0])**2 + (self.stateArray.states[i].y-self.asvPose[1])**2 < self.d_detection[i]**2):
                    obStates.states.append(self.stateArray.states[i])
        self.states_pub.publish(obStates)

    def run(self):
        self.initState()
        r = rospy.Rate(1/self.dt)
        while (not rospy.is_shutdown()):
            self.publish()
            self.update()
            r.sleep()

    def start_callback(self, data):
        self.start = True

    def _odom_callback(self, data):
        if len(self.asvPose) == 0:
            self.asvPose = np.zeros(2)
        self.asvPose[0] = data.pose.pose.position.x
        self.asvPose[1] = data.pose.pose.position.y


# Utils
def dist_init(u_d, u_d_asv, theta, t_col) :
    return (u_d_asv*t_col)**2 + (u_d*t_col)**2 - 2*u_d*u_d_asv*(t_col**2)*np.cos(theta)

def delay(u_d, u_d_asv, theta, t_col, d_detec) :
    return t_col - d_detec/np.sqrt(u_d**2 + u_d_asv**2 - 2*u_d_asv*u_d*np.cos(theta))


if __name__ == "__main__":
    rospy.init_node("obstacles_controller")

    obst = Obstacles()
    r = rospy.Rate(1/obst.dt)
    while (not obst.start and not rospy.is_shutdown()):
        r.sleep()
    obst.run()
