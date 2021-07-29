#!/usr/bin/env python3

import numpy as np
import random as rdm
import rospy
import tf
from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Obstacles(object):

    def __init__(self):
        
        self.op = rospy.get_param("opus",-1)
        self.density = rospy.get_param("~density",.01)                  #width of right lane
        self.rlw = rospy.get_param("~rlw",300.)                         #width of right lane
        self.llw = rospy.get_param("~llw",300.)                         #width of left lane
        self.ld = rospy.get_param("~ld",150.)                           #distance between lanes
        self.nOb = rospy.get_param("~nOb",20)                           #number of obstacles
        self.dDetect = rospy.get_param("~d_detection", self.nOb*[500.]) #distance of detection of obstacles
        self.prior = rospy.get_param("~prior", self.nOb*["n"])          #priority satus of obstacles
        self.size = rospy.get_param("~size", self.nOb*[8.])             #size of obstacles
        self.ll = self.rlw+self.llw+self.ld                             #length of lane
        #print(f' nOb = {self.nOb}\n rlw = {self.rlw}\n llw = {self.llw}\n ld = {self.ld}')
        #print(f'this opus: {self.op}')

        self.dt = rospy.get_param("~update_rate", .1)

        self.go = False

        self.asvPose = []
        self.obStates = []

        self.states_pub = rospy.Publisher("obstacle_states", StateArray, queue_size=1)
        self.obStatesRef_pub = rospy.Publisher("obStatesRef", StateArray, queue_size=1)
        self.br = tf.TransformBroadcaster()

        self.detect = Marker()
        self.detect.header.frame_id = "map"
        self.detect.header.stamp    = rospy.get_rostime()
        self.detect.ns = "detect"
        self.detect.id = 0
        self.detect.type = 7
        self.detect.action = 0
        self.detect.pose.orientation.x = 0
        self.detect.pose.orientation.y = 0
        self.detect.pose.orientation.z = 0
        self.detect.pose.orientation.w = 1.0
        self.detect.scale.x = 2.0
        self.detect.scale.y = 2.0
        self.detect.scale.z = 2.0
        self.detect.color.r = 2.0
        self.detect.color.g = 0
        self.detect.color.b = 0.
        self.detect.color.a = 1.0
        self.detect.lifetime = rospy.Duration(0.)
        self.detect_pub = rospy.Publisher("detect", Marker, queue_size=10, latch=True)

        self.rlane = Marker()
        self.rlane.header.frame_id = "map"
        self.rlane.header.stamp    = rospy.get_rostime()
        self.rlane.ns = "rlane"
        self.rlane.id = 0
        self.rlane.type = 4
        self.rlane.action = 0
        self.rlane.pose.orientation.x = 0
        self.rlane.pose.orientation.y = 0
        self.rlane.pose.orientation.z = 0
        self.rlane.pose.orientation.w = 1.0
        self.rlane.scale.x = 2.0
        self.rlane.scale.y = 2.0
        self.rlane.scale.z = 2.0
        self.rlane.color.r = 2.0
        self.rlane.color.g = 0
        self.rlane.color.b = 0.
        self.rlane.color.a = 1.0
        self.rlane.lifetime = rospy.Duration(100.)
        self.rlane_pub = rospy.Publisher("rlane", Marker, queue_size=10, latch=True)

        self.llane = Marker()
        self.llane.header.frame_id = "map"
        self.llane.header.stamp    = rospy.get_rostime()
        self.llane.ns = "llane"
        self.llane.id = 0
        self.llane.type = 4
        self.llane.action = 0
        self.llane.pose.orientation.x = 0
        self.llane.pose.orientation.y = 0
        self.llane.pose.orientation.z = 0
        self.llane.pose.orientation.w = 1.0
        self.llane.scale.x = 2.0
        self.llane.scale.y = 2.0
        self.llane.scale.z = 2.0
        self.llane.color.r = 2.0
        self.llane.color.g = 0
        self.llane.color.b = 0.
        self.llane.color.a = 1.0
        self.llane.lifetime = rospy.Duration(100.)
        self.llane_pub = rospy.Publisher("llane", Marker, queue_size=10, latch=True)


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

        perm = (np.random.permutation(self.nOb//2)+.5)/(self.nOb//2)
        x = (np.random.random(self.nOb)-.5)*self.ll

        for i in range(self.nOb) :
            s = State()
            s.header.id = i
            s.header.name = "Obst " + str(i)
            s.header.radius = self.size[i]
            s.header.prior = self.prior[i]
            s.u = .5*(5.+i%5)     #speed distribution
            s.v = 0.
            s.r = 0.
            s.x = x[i]

            if i%2 == 0:
                s.psi = 0.   
                s.y = self.ld/2+perm[i//2]*self.rlw   #self.ld/2+((i//2)/(self.nOb//2))*self.rlw

            else:
                s.psi = np.pi
                s.y = -self.ld/2-perm[i//2]*self.llw  #-self.ld/2-((i//2)/(self.nOb//2))*self.llw

            self.obStates.states.append(s)
        
        self.rlane.points = [Point(-self.ll/2,self.ld/2,0.),
                             Point(self.ll/2,self.ld/2,0.),
                             Point(self.ll/2,self.ld/2+self.rlw,0.),
                             Point(-self.ll/2,self.ld/2+self.rlw,0.),
                             Point(-self.ll/2,self.ld/2,0.)]
        self.llane.points = [Point(-self.ll/2,-self.ld/2,0.),
                             Point(self.ll/2,-self.ld/2,0.),
                             Point(self.ll/2,-self.ld/2-self.llw,0.),
                             Point(-self.ll/2,-self.ld/2-self.llw,0.),
                             Point(-self.ll/2,-self.ld/2,0.)]
        self.rlane_pub.publish(self.rlane)
        self.llane_pub.publish(self.llane)
    
    def incr(self,x,dx):
        return (x+dx+self.ll/2)%self.ll-self.ll/2
        
    def update(self):

        for i in range(self.nOb) :
            self.obStates.states[i].x = self.incr(self.obStates.states[i].x, 
                                        self.obStates.states[i].u*np.cos(self.obStates.states[i].psi)*self.dt)
            self.obStates.states[i].y = self.incr(self.obStates.states[i].y, 
                                        self.obStates.states[i].u*np.sin(self.obStates.states[i].psi)*self.dt)

    def publish(self):
        self.obStatesRef_pub.publish(self.obStates)
        msg = StateArray()
        self.detect.points = []
        pOb = np.zeros(2)

        for i in range(self.nOb):
            pOb[0] = self.obStates.states[i].x
            pOb[1] = self.obStates.states[i].y

            if np.linalg.norm(self.asvPose-pOb) < self.dDetect[i]:
                msg.states.append(self.obStates.states[i])
                self.detect.points.append(Point(pOb[0],pOb[1],10.))
            
            self.br.sendTransform((self.obStates.states[i].x,self.obStates.states[i].y,0),
                            tf.transformations.quaternion_from_euler(0,0,self.obStates.states[i].psi),
                            rospy.Time.now(),
                            str(self.op)+self.obStates.states[i].header.name,
                            "map")

        self.states_pub.publish(msg)
        self.detect_pub.publish(self.detect)

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
