#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from asv_msgs.msg import StateArray

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

        self.odom = np.zeros(7)
        self.n_obst = -1
        self.obst_states = []
        self.dcpa = []
        self.time_occur = []
        self.indic1 = []
        self.indic2 = []
        self.security = [] #indicateurs de sécurité pour chaque obstacle
        self.t0 = 30 #temps de sécurité en s
        self.r_offset = 0. #offset pour COLREG
        self.psi_offset = 0. #offset pour COLREG

        self.output = output
        self.opus = op

        self.finished = finished #0 : no shutdown at the end, 1 : shutdown at the end but program running, 2 : shutdown and prgrm ended

    def _odom_callback(self, data):
        vx = self.odom[2]
        vy = self.odom[3]
        print(f"odom: {vx,vy}")
        t = rospy.get_time()
        self.odom[0] = data.pose.pose.position.x
        self.odom[1] = data.pose.pose.position.y
        self.odom[2] = data.twist.twist.linear.x
        self.odom[3] = data.twist.twist.linear.y
        self.odom[4] = (data.twist.twist.linear.x-vx)/(t-self.odom[6])
        self.odom[5] = (data.twist.twist.linear.y-vy)/(t-self.odom[6])
        self.odom[6] = t

    def _obst_callback(self, data):
        if (self.n_obst == -1) :
            self.n_obst = len(data.states)
            self.dcpa = np.ones(self.n_obst)*sys.float_info.max
            self.time_occur = np.zeros(self.n_obst)
            self.indic1 = np.zeros(self.n_obst)
            self.indic2 = np.zeros(self.n_obst)
            self.obst_states = np.zeros((self.n_obst, 7))
            self.security = np.zeros((self.n_obst,4))

        for i in range(self.n_obst) :
            t = rospy.get_time()
            vx = self.obst_states[i,2]
            vy = self.obst_states[i,3]
            print(f"obst: {vx,vy}")
            self.obst_states[i, 0] = data.states[i].x
            self.obst_states[i, 1] = data.states[i].y
            self.obst_states[i, 2] = data.states[i].u*np.cos(data.states[i].psi)
            self.obst_states[i, 3] = data.states[i].u*np.sin(data.states[i].psi)
            self.obst_states[i, 4] = (self.obst_states[i, 2]-vx)/(t-self.obst_states[i, 6])
            self.obst_states[i, 5] = (self.obst_states[i, 3]-vy)/(t-self.obst_states[i, 6])
            self.obst_states[i, 6] = t



    def _start_callback(self, data):
        if (not self.start):
            self.start = True
            self.begin_sim = rospy.Time.to_sec(rospy.Time.now())
            self.begin_wall = time.time()
            print("---------------------BEGINNING OF THE SIMULATION---------------------")

    def _finish_callback(self, data) :
        f = open(f'{self.output}','a')
        print("---------------------END OF THE SIMULATION---------------------")
        print(f'Duration of the simulation (real time) : {time.time() -self.begin_wall} s')
        print(f'Duration of the simulation (simulation time): {rospy.get_time()-self.begin_sim} s')
        print(f'Number of ships : {self.n_obst}')
        for i in range(self.n_obst) :
            print(f'Ship {i+1}')
            print(f'     --> dCPA = {self.dcpa[i]} m')
            print(f'     --> t = {self.time_occur[i]} s')
            print(f'     --> indic1 = {self.indic1[i]} m/s')
            print(f'     --> indic2 = {self.indic2[i]} m/s')
            print(f'     --> indic2 = {self.security[i]}')

            if (self.opus <= 1) :
                f.write('OPUS    LOG_COL    NAT_COL    OFFSET_LOG    ANTICIPATION\n')
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
            d = self.ob_dist()
            secu = self.ob_secu()
            for i in range(self.n_obst) :
                for j in range(len(self.security[i])):
                    self.security[i, j] = np.maximum(self.security[i, j],secu[j,i])
                if (d[i] < self.dcpa[i]) :
                    self.dcpa[i] = d[i]
                    #norm_ot = np.array([self.odom[0] - self.obst_states[i, 0], self.odom[1] - self.obst_states[i, 1]])
                    #norm_ot = self.obst_states[i,:2] - self.odom[:2]
                    #norm_ot /= np.linalg.norm(norm_ot)
                    #aux = np.array([self.obst_states[i, 2]*np.cos(self.obst_states[i, 3]), self.obst_states[i, 2]*np.sin(self.obst_states[i, 3])])
                    #aux2 = np.array([self.odom[2], self.odom[3]])
                    #self.indic1[i] = - np.dot(aux, norm_ot)
                    #self.indic2[i] = np.dot(norm_ot, aux2)
                    self.time_occur[i] = rospy.get_time() - self.begin_sim
        #self.sim_time += self.rate

    def ob_dist(self) :
        dist = np.zeros(self.n_obst)
        for i in range(self.n_obst) :
            dist[i] = np.linalg.norm(self.obst_states[i,2]-self.odom[:2]) # beware of the map resolution
        return dist

    def ob_secu(self) :
        dist = self.ob_dist()
        rvel = np.zeros(self.n_obst) #relative velocity
        acc = np.zeros(self.n_obst) #relative acceleration
        offd = np.zeros(self.n_obst) #distance avec offset
        asv_psi = np.arctan2(self.odom[3],self.odom[2])
        obst_psi = np.arctan2(self.obst_states[:,3],self.odom[2])
        for i in range(self.n_obst) :
            rvel[i] = np.linalg.norm(self.obst_states[i,2:4]-self.odom[2:4])
            acc[i] = np.linalg.norm(self.odom[4:6])#-self.obst_states[i,4:6])
            asv_off = self.odom[:2]+np.array([self.r_offset*np.cos(asv_psi-self.psi_offset),
                                              self.r_offset*np.sin(asv_psi-self.psi_offset)])
            obst_off = self.obst_states[i,:2]+np.array([self.r_offset*np.cos(obst_psi[i]+self.psi_offset),
                                                        self.r_offset*np.sin(obst_psi[i]+self.psi_offset)])
            offd[i] = np.linalg.norm(asv_off-obst_off) # beware of the map resolution
        return np.array([np.log(self.t0*rvel/dist),  #indicateur logarithmique de collision
                         self.t0*rvel/dist,          #indicateur naturel de collision
                         np.log(self.t0*rvel/offd),  #indicateur logarithmique de collision avec offset
                         acc/dist])                 #indicateur d'anticipation


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

        #def h() :
        #    print "Shutting down..."
        #rospy.on_shutdown(h)
        rospy.signal_shutdown("End of the simulation")


if __name__ == "__main__" :

    rospy.init_node("Referee")

    dt = 1/rospy.get_param("~update_rate", 5.)
    finished = rospy.get_param("~shutdown", 0)
    output = rospy.get_param("~output_file", '/home/adrien/catkin/src/seaowl/asv_system/output/test.txt')
    op = rospy.get_param("~opus", '0')

    print(f'Output : {output}')

    refer = Referee(dt, finished, output, op)

    msg  = Empty()
    refer.start_publisher.publish(msg) #top départ (temporary)

    refer.run_controller()
