#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from asv_msgs.msg import StateArray
from visualization_msgs.msg import Marker
from scipy.signal import savgol_filter as sgf
import os

import time

import sys

class Referee(object) :

    def __init__(self, dt=0.2, finished=0,
                 output='/home/adrien/catkin/src/seaowl/asv_system/output',
                 op='0') :

        self.debugBool = False   #if True prints the trajectory (t,x,y) of asv in debug.txt
        self.rvizBool = False

        self.begin_wall = 0.
        self.begin_sim = 0.
        self.start = False
        self.rate = 1/dt

        self.odom = []
        self.n_obst = -1
        self.obst_prior = []
        self.obst_radii = []
        self.obst_states = []
        self.dcpa = []
        self.tcpa = []
        self.cross = []
        self.security = []        #indicateurs de sécurité pour chaque obstacle
        self.t0 = 4.              #temps de sécurité en s
        self.d0 = 1/100           #distance minimale
        self.t1 = 10              #temps de manoeuvre
        self.d1 = 500             #distance de manoeuvre
        self.tth = 75             #temps théorique de manoeuvre
        self.r_offset = 5.        #offset pour COLREG
        self.size = 8.            #asv size radius
        self.output = output
        self.opus = op
        self.finished = finished  #0 : no shutdown at the end, 1 : shutdown at the end but program running, 2 : shutdown and prgrm ended
        self.side = []
        if self.debugBool:
            self.debug = open(f'/home/adrien/catkin_ws/src/seaowl/asv_system/debug.txt','w')
        self.traj = []
        self.nend = True

        if self.rvizBool:
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
            self.cpa_publisher = rospy.Publisher("/cpa", Marker, queue_size=10, latch=True)

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
            self.cpa2_publisher = rospy.Publisher("/cpa2", Marker, queue_size=10, latch=True)

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
            self.asv_off_marker.pose.orientation.w = 1.
            self.asv_off_marker.scale.x = 1.0
            self.asv_off_marker.scale.y = 1.0
            self.asv_off_marker.scale.z = 1.0
            self.asv_off_marker.color.r = 0.5
            self.asv_off_marker.color.g = 0.0
            self.asv_off_marker.color.b = 0.5
            self.asv_off_marker.color.a = 1.0
            self.asv_off_marker.lifetime = rospy.Duration(0.)
            self.asv_off_publisher = rospy.Publisher("/asv_off", Marker, queue_size=10, latch=True)

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
            self.obst_off_publisher = rospy.Publisher("obst_off", Marker, queue_size=10, latch=True)

        self.start_publisher   = rospy.Publisher("start_simulation", Empty, queue_size=1, latch=True)
        self.asv_off_publisher = rospy.Publisher("asv_off", Marker, queue_size=10, latch=True)
        self.obst_off_publisher = rospy.Publisher("obst_off", Marker, queue_size=10, latch=True)
        self.cpa_publisher = rospy.Publisher("cpa", Marker, queue_size=10, latch=True)
        self.cpa2_publisher = rospy.Publisher("cpa2", Marker, queue_size=10, latch=True)

        self._odom_subscriber = rospy.Subscriber("state", Odometry,
                                                    self._odom_callback,
                                                    queue_size=1)
        self._obst_subscriber = rospy.Subscriber("obstacle_states",
                                                    StateArray, self._obst_callback,
                                                    queue_size=1)
        self._finish_subscriber = rospy.Subscriber("end_simulation",
                                                    Empty, self._finish_callback,
                                                    queue_size=1)
        self._start_subscriber = rospy.Subscriber("start_simulation", Empty,
                                                    self._start_callback,
                                                    queue_size=10)

    def _odom_callback(self, data):
        if self.nend:
            if len(self.odom) == 0:
                self.odom = np.zeros(5)
                t = rospy.get_time()-self.begin_sim
                self.odom[0] = data.pose.pose.position.x
                self.odom[1] = data.pose.pose.position.y
                self.odom[4] = t
            else:
                t = rospy.get_time()-self.begin_sim
                x = self.odom[0]
                y = self.odom[1]
                vx = self.odom[2]
                vy = self.odom[3]
                self.odom[0] = data.pose.pose.position.x
                self.odom[1] = data.pose.pose.position.y
                self.odom[2] = (self.odom[0]-x)/(t-self.odom[4])
                self.odom[3] = (self.odom[1]-y)/(t-self.odom[4])
                self.odom[4] = t
                self.traj.append(np.array([t,self.odom[0],self.odom[1]]))
                if self.debugBool:
                    self.debug.write(f'{t}\t{self.odom[0]}\t{self.odom[1]}\n')
                    #print(f'add {t}')
            #print(f'uAsv = {np.linalg.norm(np.array([data.twist.twist.linear.x,data.twist.twist.linear.y]))}')
    def _obst_callback(self, data):
        if self.nend:
            if (self.n_obst == -1) :
                self.n_obst = len(data.states)
                self.dcpa = np.ones(self.n_obst)*sys.float_info.max
                self.tcpa = np.zeros((self.n_obst,2))
                self.cross = np.array(self.n_obst*[-1])
                self.obst_states = np.zeros((self.n_obst, 4))
                self.obst_radii = np.zeros(self.n_obst)
                self.security = np.zeros((self.n_obst,15))
                self.side = np.zeros(self.n_obst)
                self.obst_prior = np.array(self.n_obst*[""])
                for i in range(self.n_obst):
                    self.obst_radii[i] = data.states[i].header.radius
                    self.obst_prior[i] = data.states[i].header.prior
                    for j in range(1,4):
                        self.security[i,j] = -np.inf

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
        self.nend = False
        if self.debugBool:
            self.debug.close()
        tf = (rospy.get_time()-self.begin_sim-self.tth)/self.tth
        for i in range(self.n_obst):
            self.traj = np.array(self.traj)
            w = 11
            d = 2
            N = int(self.tcpa[i,1])
            pos = self.traj[:N,1:]
            vel = np.zeros((N,2))
            acc = np.zeros((N,2))
            for k in range(2):                                     #coordinates
                pos[:,k] = sgf(pos[:,k],w,d, mode='nearest')
                vel[:,k] = np.gradient(pos[:,k], self.traj[:N,0])
                vel[:,k] = sgf(vel[:,k],w,d-1, mode='nearest')
                acc[:,k] = np.gradient(vel[:,k], self.traj[:N,0])
                acc[:,k] = sgf(acc[:,k],w,d-2, mode='nearest')
            v = np.linalg.norm(vel,axis = 1)
            irr = np.zeros((N,3))                                   #irregularity indicator
            irr[:,0] = np.linalg.norm(acc,axis = 1)                 #acceleration
            irr[:,1] = (vel[:,0]*acc[:,1]-vel[:,1]*acc[:,0])/v**2   #angular velocity
            irr[:,2] = irr[:,1]/v                                   #curvature radius
            weight = att((np.abs(self.tcpa[i,0] -self.traj[:N,0]))/self.t1,3)
            weight = np.sqrt(weight/np.sum(weight))
            for k in range(3):
                self.security[i][4+k] = np.linalg.norm(irr[:,k]*weight)
                self.security[i,7+k] = self.security[i,1]+max(0,self.security[i,1])*self.security[i][4+k]
            self.security[i,0] = tf
            self.security[i,10] = self.dcpa[i]
            self.security[i,11] = self.cross[i]
            weight = irr[:,0]**2/np.sum(irr[:,0]**2)
            self.security[i,12] = np.dot(weight,(np.abs(self.tcpa[i,0] -self.traj[:N,0])/self.t1))
            self.security[i,13] = 1/self.security[i,2]+max(1/self.security[i,2]-1,0)*self.security[i,12]

        f = open(f'{self.output}','a')
        print("---------------------END OF THE SIMULATION---------------------")
        print(f'Duration of the simulation (real time) : {time.time() -self.begin_wall} s')
        print(f'Relative duration of the simulation: {tf} s')
        print(f'Number of ships : {self.n_obst}')
        for i in range(self.n_obst) :
            print(f'Ship {i+1}')
            print(f'     --> dCPA = {self.dcpa[i]} m')
            print(f'     --> tCPA = {self.tcpa[i,0]} s')
            print(f'     --> security = {self.security[i]}')

        if (os.stat(self.output).st_size == 0) :
            f.write('OPUS    TIME    LOG_COL    NAT_COL    OFFSET_LOG    ANTICIPATION_ACC    ANTICIPATION_OMEGA    ANTICIPATION_R    AGG_ACC    AGG_OMEGA    AGG_R    DCPA    CROSSING_DIST    ANT_TIME    AGG_TIME    N_CROSS\n')
        f.write(f'{self.opus}')
        for sec_indic in range(len(self.security[0])) :
            f.write(f'    {np.max(self.security[:,sec_indic])}')
        f.write(f'\n')
        f.close()
        print(f'Output logged in {self.output}')
        print("---------------------------------------------------------------")
        if self.finished == 1 :
            self.finished = 2

    def _update(self):
        if (self.n_obst > -1 and len(self.odom) > 0) :
            secu = self.ob_secu()
            for i in range(self.n_obst) :
                for j in range(1,4):
                    self.security[i, j] = max(self.security[i, j],secu[i,j])
                if (secu[i,0] <= self.dcpa[i]) :
                    self.dcpa[i] = secu[i,0]
                    self.tcpa[i,0] = rospy.get_time() - self.begin_sim
                    self.tcpa[i,1] = len(self.traj)

                    if self.rvizBool:

                        self.cpa.pose.position.x = self.odom[0]
                        self.cpa.pose.position.y = self.odom[1]
                        self.cpa2.pose.position.x = self.obst_states[i,0]
                        self.cpa2.pose.position.y = self.obst_states[i,1]

                if self.rvizBool:

                    self.cpa_publisher.publish(self.cpa)
                    self.cpa2_publisher.publish(self.cpa2)

                side = np.dot(self.odom[0:2]-self.obst_states[i,0:2],rot(self.obst_states[i,2:4],np.pi/2))
                front  = np.dot(self.odom[0:2]-self.obst_states[i,0:2],self.obst_states[i,2:4])
                if (side*self.side[i] < 0 and front > 0) :
                    self.cross[i] = secu[i,0]
                    self.security[i,14] += att(secu[i,0]/self.d1,3)
                self.side[i] = side



    def ob_dist(self) :
        dist = np.zeros(self.n_obst)
        for i in range(self.n_obst) :
            dist[i] = max(0,np.linalg.norm(self.obst_states[i,:2]-self.odom[:2])
                            -self.size-self.obst_radii[i])
        return dist

    def ob_secu(self) :
        dist = self.ob_dist()
        rvel = np.zeros(self.n_obst)                                               #relative velocity
        offd = np.zeros(self.n_obst)                                               #distance avec offset

        for i in range(self.n_obst) :
            rvel[i] = np.linalg.norm(self.obst_states[i,2:4]-self.odom[2:4])
            cpa = rot((self.obst_states[i,2:4]-self.odom[2:4])/rvel,np.pi/2)
            self.r_offset = dist[i]/4
            if np.dot(cpa,rot(self.odom[2:4],np.pi/2))>0 :
                cpa = -cpa
            if self.obst_prior[i] == "g":
                if np.dot(cpa,self.odom[2:4]) < 0:
                    cpa = -cpa
                asv_off = self.odom[:2]+self.r_offset*cpa                          #off before asv

                obst_off = self.obst_states[i,:2]-self.r_offset*cpa
            elif self.obst_prior[i] == "s":
                if np.dot(cpa,self.odom[2:4]) > 0:
                    cpa = -cpa
                asv_off = self.odom[:2]+self.r_offset*cpa                          #off behind asv
                obst_off = self.obst_states[i,:2]-self.r_offset*cpa
            else:
                if np.dot(self.obst_states[i,2:4],rot(self.odom[2:4],np.pi/2))>0 : #obst right of asv
                    if np.dot(cpa,self.odom[2:4]) > 0:
                        cpa = -cpa
                    asv_off = self.odom[:2]+self.r_offset*cpa                      #off behind asv
                    obst_off = self.obst_states[i,:2]-self.r_offset*cpa
                else :                                                             #obst left of asv
                    if np.dot(cpa,self.odom[2:4]) < 0:
                        cpa = -cpa
                    asv_off = self.odom[:2]+self.r_offset*cpa                      #off before asv
                    obst_off = self.obst_states[i,:2]-self.r_offset*cpa

            if self.rvizBool:

                self.asv_off_marker.pose.position.x = asv_off[0]
                self.asv_off_marker.pose.position.y = asv_off[1]
                self.asv_off_publisher.publish(self.asv_off_marker)
                self.obst_off_marker.pose.position.x = obst_off[0]
                self.obst_off_marker.pose.position.y = obst_off[1]
                self.obst_off_publisher.publish(self.obst_off_marker)

            offd[i]= min(max(0,np.linalg.norm(asv_off-obst_off)-self.size-self.obst_radii[i]),
                         max(0,np.linalg.norm(self.odom[:2]-obst_off)-self.size-self.obst_radii[i]),
                         max(0,np.linalg.norm(asv_off-self.obst_states[i,:2])-self.size-self.obst_radii[i]),
                         dist[i])

        secu = np.zeros((self.n_obst,4))
        secu[:,0] = dist                                                  #distance
        secu[:,1] = np.log(self.t0*rvel*att(dist/self.d0,0)/self.d0)      #indicateur logarithmique de collision
        secu[:,2] = self.t0*rvel*att(dist/self.d0,0) /self.d0             #indicateur naturel de collision
        secu[:,3] = np.log(self.t0*rvel*att(offd/self.d0,0)/self.d0)      #indicateur logarithmique de collision avec offset

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


#utils
def att(x,option) :
    if option == 0:
        return np.minimum(1,1/x)
    if option == 1:
        return 1/(1+x)
    if option == 2:
        return np.exp(-x)
    if option == 3:
        return np.maximum(1-x,0)


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
