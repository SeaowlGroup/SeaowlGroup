#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray
from std_msgs.msg import Empty

class Obstacles(object):

    def __init__(self):
        
        self.op = rospy.get_param("opus",-1)
        self.nOb = rospy.get_param("nOb",20)                            #number of obstacles
        self.dDetect = rospy.get_param("~d_detection", self.nOb*[500.]) #distance of detection of obstacles
        self.prior = rospy.get_param("~prior", self.nOb*["n"])          #priority satus of obstacles
        self.size = rospy.get_param("~size", self.nOb*[8.])             #size of obstacles
        #self.nl = rospy.get_param("nl",2)                              #number of lanes
        self.rlw = rospy.get_param("rlw",300.)                          #width of right lane
        self.llw = rospy.get_param("rlw",300.)                          #width of left lane
        self.ll = 900.                                                  #length of lane
        self.ld = rospy.get_param("ld",150.)                            #distance between lanes

        #print(f'this opus: {self.op}')

        self.dt = rospy.get_param("~update_rate", .1)

        self.go = False

        self.asvPose = []
        self.obStates = []

        self.states_pub = rospy.Publisher("obstacle_states", StateArray, queue_size=1)


        self._start_subscriber = rospy.Subscriber("start_simulation", Empty,
                                                self._start_callback,
                                                queue_size=10)
        self._odom_subscriber = rospy.Subscriber("state", Odometry,
                                                    self._odom_callback,
                                                    queue_size=1)
        self._end_subscriber = rospy.Subscriber("end_simulation", Empty,
                                                self._end_callback,
                                                queue_size=10)

    def initObs(self):
        self.obStates = StateArray()

        for i in range(self.nOb) :
            s = State()
            s.header.id = i
            s.header.name = "Obst " + str(i)
            s.header.radius = self.size[i]
            s.header.prior = self.prior[i]
            s.u = .5*(5.+i%5)     #speed distribution
            s.v = 0.
            s.r = 0.

            if i%2 == 0:
                s.psi = 0.   
                s.x = ((i//2)/(self.nOb//2)-.5)*self.ll
                s.y = self.ld/2+((i//2)/(self.nOb//2))*self.rlw


            else:
                s.psi = np.pi
                s.x = ((i//2)/(self.nOb//2)-.5)*self.ll
                s.y = self.ld/2-((i//2)/(self.nOb//2))*self.llw

            self.obStates.states.append(s)
    
    def incr(self,x,dx):
        return (x+dx+self.ll/2)%self.ll-self.ll/2
        
    def update(self):

        for i in range(self.nOb) :
            self.obStates.states[i].x = self.incr(self.obStates.states[i].x, 
                                        self.obStates.states[i].u*np.cos(self.obStates.states[i].psi)*self.dt)
            self.obStates.states[i].y = self.incr(self.obStates.states[i].y, 
                                        self.obStates.states[i].u*np.sin(self.obStates.states[i].psi)*self.dt)

    def publish(self):
        msg = StateArray()
        pOb = np.zeros(2)
        br = tf.TransformBroadcaster()

        for i in range(self.nOb):
            pOb[0] = self.obStates.states[i].x
            pOb[1] = self.obStates.states[i].y

            if np.linalg.norm(self.asvPose-pOb) < self.dDetect[i]:
                msg.states.append(self.obStates.states[i])
            
            br.sendTransform((self.obStates.states[i].x,self.obStates.states[i].y,0),
                            tf.transformations.quaternion_from_euler(0,0,self.obStates.states[i].psi),
                            rospy.Time.now(),
                            str(self.op)+self.obStates.states[i].header.name,
                            "map")

        self.states_pub.publish(msg)

    def run(self):
        r = rospy.Rate(1/self.dt)

        while (not rospy.is_shutdown()):
            if self.go:
                self.update()
                self.publish() 
            r.sleep()

    def _start_callback(self, data):
        self.go = True

    def _odom_callback(self, data):

        if len(self.asvPose) == 0:
            self.asvPose = np.zeros(2)

        self.asvPose[0] = data.pose.pose.position.x
        self.asvPose[1] = data.pose.pose.position.y
    
    def _end_callback(self, data):
        self.go = False


if __name__ == "__main__":
    rospy.init_node("obstacles_controller")

    obst = Obstacles()
    obst.initObs()
    obst.run()
