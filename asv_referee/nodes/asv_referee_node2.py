#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from asv_msgs.msg import StateArray
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt

import time

import sys

class Referee(object) :

    def __init__(self, dt=0.2, finished=0,
                 output='/home/adrien/catkin/src/seaowl/asv_system/output',
                 op='0') :

        self.begin_wall = 0.
        self.begin_sim = 0.
        self.start = False
        self.rate = 1/dt

        self._odom_subscriber = rospy.Subscriber("/asv/state", Odometry,
                                                    self._odom_callback,
                                                    queue_size=1)
        self._obst_subscriber = rospy.Subscriber("/obstacle_states",
                                                    StateArray, self._obst_callback,
                                                    queue_size=1)
        self._finish_subscriber = rospy.Subscriber("/end_simulation",
                                                    Empty, self._finish_callback,
                                                    queue_size=1)
        self._start_subscriber = rospy.Subscriber("/start_simulation", Empty,
                                                    self._start_callback,
                                                    queue_size=10)
        self.start_publisher   = rospy.Publisher("/start_simulation", Empty, queue_size=1, latch=True)
        self.asv_off_publisher = rospy.Publisher("/asv_off", Marker, queue_size=10, latch=True)
        self.obst_off_publisher = rospy.Publisher("/obst_off", Marker, queue_size=10, latch=True)
        self.cpa_publisher = rospy.Publisher("/cpa", Marker, queue_size=10, latch=True)
        self.cpa2_publisher = rospy.Publisher("/cpa2", Marker, queue_size=10, latch=True)

        self.odom = []
        self.n_obst = -1
        self.obst_states = []
        self.dcpa = []
        self.tcpa = []
        self.cross = []
        self.security = []        #indicateurs de sécurité pour chaque obstacle
        self.t0 = 30              #temps de sécurité en s
        self.d0 = 200             #distance de manoeuvre
        self.d1 = 1               #distance minimale
        self.r_offset = 5.        #offset pour COLREG
        self.size = 8.            #asv size radius
        self.output = output
        self.opus = op
        self.finished = finished  #0 : no shutdown at the end, 1 : shutdown at the end but program running, 2 : shutdown and prgrm ended
        self.side = []
        self.obst_prior = []
        self.debug = open(f'/home/adrien/catkin_ws/src/seaowl/asv_system/debug.txt','w')
        self.traj = []

        self.cpa = Marker()
        self.cpa.header.frame_id = "map"
        self.cpa.header.stamp    = rospy.get_rostime()
        self.cpa.ns = "cpa"
        self.cpa.id = 0
        self.cpa.type = 1
        self.cpa.action = 0
        self.cpa.pose.position.x = 0
        self.cpa.pose.position.y = 0
        self.cpa.pose.position.z = 10
        self.cpa.pose.orientation.x = 0
        self.cpa.pose.orientation.y = 0
        self.cpa.pose.orientation.z = 0
        self.cpa.pose.orientation.w = 1.0
        self.cpa.scale.x = 1.0
        self.cpa.scale.y = 1.0
        self.cpa.scale.z = 1.0
        self.cpa.color.r = 1.0
        self.cpa.color.g = 0
        self.cpa.color.b = 0.
        self.cpa.color.a = 1.0
        self.cpa.lifetime = rospy.Duration(0.)


        self.cpa2 = Marker()
        self.cpa2.header.frame_id = "map"
        self.cpa2.header.stamp    = rospy.get_rostime()
        self.cpa2.ns = "cpa2"
        self.cpa2.id = 1
        self.cpa2.type = 1
        self.cpa2.action = 0
        self.cpa2.pose.position.x = 0
        self.cpa2.pose.position.y = 0
        self.cpa2.pose.position.z = 10
        self.cpa2.pose.orientation.x = 0
        self.cpa2.pose.orientation.y = 0
        self.cpa2.pose.orientation.z = 0
        self.cpa2.pose.orientation.w = 1.0
        self.cpa2.scale.x = 1.0
        self.cpa2.scale.y = 1.0
        self.cpa2.scale.z = 1.0
        self.cpa2.color.r = 1.0
        self.cpa2.color.g = 0
        self.cpa2.color.b = 0.
        self.cpa2.color.a = 1.0
        self.cpa2.lifetime = rospy.Duration(0.)

        self.asv_off_marker = Marker()
        self.asv_off_marker.header.frame_id = "map"
        self.asv_off_marker.header.stamp    = rospy.get_rostime()
        self.asv_off_marker.ns = "asv_off_marker"
        self.asv_off_marker.id = 2
        self.asv_off_marker.type = 2
        self.asv_off_marker.action = 0
        self.asv_off_marker.pose.position.x = 0.
        self.asv_off_marker.pose.position.y = 0.
        self.asv_off_marker.pose.position.z = 0
        self.asv_off_marker.pose.orientation.x = 0
        self.asv_off_marker.pose.orientation.y = 0
        self.asv_off_marker.pose.orientation.z = 0
        self.asv_off_marker.pose.orientation.w = 1.0
        self.asv_off_marker.scale.x = 1.0
        self.asv_off_marker.scale.y = 1.0
        self.asv_off_marker.scale.z = 1.0
        self.asv_off_marker.color.r = 0.5
        self.asv_off_marker.color.g = 0.0
        self.asv_off_marker.color.b = 0.5
        self.asv_off_marker.color.a = 1.0
        self.asv_off_marker.lifetime = rospy.Duration(0.)

        self.obst_off_marker = Marker()
        self.obst_off_marker.header.frame_id = "map"
        self.obst_off_marker.header.stamp    = rospy.get_rostime()
        self.obst_off_marker.ns = "obst_off_marker"
        self.obst_off_marker.id = 3
        self.obst_off_marker.type = 2
        self.obst_off_marker.action = 0
        self.obst_off_marker.pose.position.x = 0
        self.obst_off_marker.pose.position.y = 0
        self.obst_off_marker.pose.position.z = 0
        self.obst_off_marker.pose.orientation.x = 0
        self.obst_off_marker.pose.orientation.y = 0
        self.obst_off_marker.pose.orientation.z = 0
        self.obst_off_marker.pose.orientation.w = 1.0
        self.obst_off_marker.scale.x = 1.0
        self.obst_off_marker.scale.y = 1.0
        self.obst_off_marker.scale.z = 1.0
        self.obst_off_marker.color.r = 0.5
        self.obst_off_marker.color.g = 0.0
        self.obst_off_marker.color.b = 0.5
        self.obst_off_marker.color.a = 1.0
        self.obst_off_marker.lifetime = rospy.Duration(0.)

    def _odom_callback(self, data):
        if len(self.odom) == 0:
            self.odom = np.zeros(7)
            t = rospy.get_time()-self.begin_sim
            self.odom[0] = data.pose.pose.position.x
            self.odom[1] = data.pose.pose.position.y
            self.odom[4] = t
        else : 
            t = rospy.get_time()-self.begin_sim
            x = self.odom[0]
            y = self.odom[1]
            vx = self.odom[2]    
            vy = self.odom[3]
            self.odom[0] = data.pose.pose.position.x
            self.odom[1] = data.pose.pose.position.y
            self.odom[2] = (self.odom[0]-x)/(t-self.odom[4])
            self.odom[3] = (self.odom[1]-y)/(t-self.odom[4])
            self.odom[5] = (self.odom[2]-vx)/(t-self.odom[4])
            self.odom[6] = (self.odom[3]-vy)/(t-self.odom[4])
            self.odom[4] = t
            self.debug.write(f'{t}\t{self.odom[0]}\t{self.odom[1]}\t{self.odom[2]}\t{self.odom[3]}\t{self.odom[5]}\t{self.odom[6]}\n')

    def _obst_callback(self, data):
        if (self.n_obst == -1) :
            self.n_obst = len(data.states)
            self.dcpa = np.ones(self.n_obst)*sys.float_info.max
            self.tcpa = np.zeros(self.n_obst)
            self.cross = np.array(self.n_obst*[-1])
            self.obst_states = np.zeros((self.n_obst, 6))
            self.security = np.zeros((self.n_obst,5))
            #self.security[:,9] = np.array(self.n_obst*[-1.])
            self.side = np.zeros(self.n_obst)
            self.obst_prior = np.array(self.n_obst*[""])
            self.traj = self.n_obst*[None]
            for i in range(self.n_obst):
                self.obst_states[i,5] = data.states[i].header.radius
                self.obst_prior[i] = data.states[i].header.prior
                self.traj[i] = []

        for i in range(self.n_obst) :
            self.obst_states[i, 0] = data.states[i].x
            self.obst_states[i, 1] = data.states[i].y
            self.obst_states[i, 2] = data.states[i].u*np.cos(data.states[i].psi)
            self.obst_states[i, 3] = data.states[i].u*np.sin(data.states[i].psi)            

    def _start_callback(self, data):
        if (not self.start):
            self.start = True
            self.begin_sim = rospy.Time.to_sec(rospy.Time.now())
            self.begin_wall = time.time()
            print("---------------------BEGINNING OF THE SIMULATION---------------------")

    def _finish_callback(self, data) :
        #for i in range(self.n_obst):
            #self.traj[i] = np.array[self.traj[i]]
        #for k in range(4):
            #self.security[:,4+k] = np.sqrt(self.security[:,4+k]/self.obst_states[:,4])
        #self.security[:,0] = time.time() -self.begin_wall
        f = open(f'{self.output}','a')
        print("---------------------END OF THE SIMULATION---------------------")
        print(f'Duration of the simulation (real time) : {time.time() -self.begin_wall} s')
        print(f'Duration of the simulation (simulation time): {rospy.get_time()-self.begin_sim} s')
        print(f'Number of ships : {self.n_obst}')
        for i in range(self.n_obst) :
            print(f'Ship {i+1}')
            print(f'     --> dCPA = {self.dcpa[i]} m')
            print(f'     --> t = {self.tcpa[i]} s')
            print(f'     --> security = {self.security[i]}')

            if (self.opus <= 1) :
                f.write('OPUS    TIME   LOG_COL    NAT_COL    OFFSET_LOG    ANTICIPATION_INV   ANTICIPATION_OFF    ANTICIPATION_LIN    ANTICIPATION_EXP    DCPA    CROSSING_DIST\n')
            f.write(f'{self.opus}')
            for sec_indic in range(len(self.security[0])) :
                f.write(f'    {np.max(self.security[:,sec_indic])}')
        f.write(f'\n')
        f.close()
        print(f'Output logged in {self.output}')
        print("---------------------------------------------------------------")
        #Other indic : linear speed of the obstacle/asv scalar the vector between the two poses
        #write in an extern file
        if self.finished == 1 :
            self.finished = 2

    def _update(self):
        if (self.n_obst > -1) :
            secu = self.ob_secu()
            for i in range(self.n_obst) :
                for j in range(1,4):
                    self.security[i, j] = max(self.security[i, j],secu[i,j])
                if (secu[i,0] < self.dcpa[i]) :
                    self.dcpa[i] = secu[i,0]
                    t = rospy.get_time()
                    self.tcpa[i] = t - self.begin_sim
                    #self.obst_states[i,4] += 1
                    #for k in range(4):
                        #self.security[i,4+k] += secu[i,4+k]**2
                    #self.security[i,8] = d[i]

                    #self.traj[i].append(np.array([t,self.odom[0],self.odom[1],self.obst_states[i,0],self.obst_states[i,1]]))
                    self.cpa.pose.position.x = self.odom[0]
                    self.cpa.pose.position.y = self.odom[1]
                    self.cpa2.pose.position.x = self.obst_states[i,0]
                    self.cpa2.pose.position.y = self.obst_states[i,1]
                #else:
                    #self.debug.close()

                self.cpa_publisher.publish(self.cpa)
                self.cpa2_publisher.publish(self.cpa2)

                side = np.dot(self.odom[0:2]-self.obst_states[i,0:2],rot(self.obst_states[i,2:4],np.pi/2))
                front  = np.dot(self.odom[0:2]-self.obst_states[i,0:2],self.obst_states[i,2:4])
                if (side*self.side[i] < 0 and front > 0) :
                    self.cross[i] = secu[i,0]
                self.side[i] = side



    def ob_dist(self) :
        dist = np.zeros(self.n_obst)
        for i in range(self.n_obst) :
            dist[i] = max(0,np.linalg.norm(self.obst_states[i,:2]-self.odom[:2])
                            -self.size-self.obst_states[i,5])
        return dist

    def ob_secu(self) :
        dist = self.ob_dist()
        rvel = np.zeros(self.n_obst)            #relative velocity
        #acc = np.linalg.norm(self.odom[4:6])    #asv acceleration
        offd = np.zeros(self.n_obst)            #distance avec offset

        for i in range(self.n_obst) :
            rvel[i] = np.linalg.norm(self.obst_states[i,2:4]-self.odom[2:4])
            cpa = rot((self.obst_states[i,2:4]-self.odom[2:4])/rvel,np.pi/2)
            if np.dot(cpa,rot(self.odom[2:4],np.pi/2))>0 :
                cpa = -cpa
            if self.obst_prior[i] == "g":
                if np.dot(cpa,self.odom[2:4]) < 0:
                    cpa = -cpa 
                asv_off = self.odom[:2]+self.r_offset*cpa   #off before asv
                obst_off = self.obst_states[i,:2]-self.r_offset*cpa  
            elif self.obst_prior[i] == "s":
                if np.dot(cpa,self.odom[2:4]) > 0:
                    cpa = -cpa
                asv_off = self.odom[:2]+self.r_offset*cpa   #off behind asv 
                obst_off = self.obst_states[i,:2]-self.r_offset*cpa  
            else:
                if np.dot(self.obst_states[i,2:4],rot(self.odom[2:4],np.pi/2))>0 : #obst right of asv
                    if np.dot(cpa,self.odom[2:4]) > 0:
                        cpa = -cpa
                    asv_off = self.odom[:2]+self.r_offset*cpa   #off behind asv 
                    obst_off = self.obst_states[i,:2]-self.r_offset*cpa
                else : #obst left of asv
                    if np.dot(cpa,self.odom[2:4]) < 0:
                        cpa = -cpa
                    asv_off = self.odom[:2]+self.r_offset*cpa   #off before asv 
                    obst_off = self.obst_states[i,:2]-self.r_offset*cpa
            self.asv_off_marker.pose.position.x = asv_off[0]
            self.asv_off_marker.pose.position.y = asv_off[1]
            self.asv_off_publisher.publish(self.asv_off_marker)
            self.obst_off_marker.pose.position.x = obst_off[0]
            self.obst_off_marker.pose.position.y = obst_off[1]
            self.obst_off_publisher.publish(self.obst_off_marker)

            offd[i]= min(max(0,np.linalg.norm(asv_off-obst_off)-self.size-self.obst_states[i,5]),
                         dist[i])
        secu = np.zeros((self.n_obst,4))
        secu[:,0] = dist
        secu[:,1] = np.log(self.t0*rvel*self.dist_att(dist,4))   #indicateur logarithmique de collision
        secu[:,2] = self.t0*rvel*self.dist_att(dist,4)           #indicateur naturel de collision 
        secu[:,3] = np.log(self.t0*rvel*self.dist_att(offd,4))

        #if dist == 0:
            #secu[:,1] = 10              #indicateur logarithmique de collision
            #secu[:,2] = 250             #indicateur naturel de collision
            #secu[:,4] = 30
        #else:
        #secu[:,4] = np.minimum(30, acc*self.dist_att(dist,0))
        #if offd == 0:
            #secu[:,3] = 10
        #else:
        #secu[:,5] = acc*self.dist_att(dist,1)
        #secu[:,6] = acc*self.dist_att(dist,2)
        #secu[:,7] = acc*self.dist_att(dist,3)
        return secu


    def run_controller(self):
        r = rospy.Rate(self.rate)
        while (not rospy.is_shutdown()) and self.finished < 2 :
            self._update()
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.is_shutdown():
                    break
                raise
        rospy.signal_shutdown("End of the simulation")

    def dist_att(self,d,option) :
        if option == 0:
            return self.d0/d
        if option == 1:
            return 1/(1+d/self.d0)
        if option == 2:
            return np.maximum(1-d/self.d0,0)
        if option == 3:
            return np.exp(-d/self.d0)
        if option == 4:
            return np.minimum(1/self.d1,1/d)

def rot(u,phi):
    v = np.zeros(2)
    v[0] = np.cos(phi)*u[0]-np.sin(phi)*u[1]
    v[1] = np.sin(phi)*u[0]+np.cos(phi)*u[1]
    return v

if __name__ == "__main__" :

    rospy.init_node("Referee")

    dt = 1/rospy.get_param("~update_rate", 15.)
    finished = rospy.get_param("~shutdown", 0)
    output = rospy.get_param("~output_file", '/home/adrien/catkin/src/seaowl/asv_system/output/test.txt')
    op = rospy.get_param("~opus", '0')

    print(f'Output : {output}')

    refer = Referee(.01, finished, output, op)

    msg  = Empty()
    refer.start_publisher.publish(msg) #top départ (temporary)

    refer.run_controller()
