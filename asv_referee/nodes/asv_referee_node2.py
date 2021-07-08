#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from asv_msgs.msg import StateArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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


        self.odom = np.zeros(7)
        self.n_obst = -1
        self.obst_states = []
        self.obst_front = []
        self.dcpa = []
        self.time_occur = []
        self.indic1 = []
        self.indic2 = []
        self.security = [] #indicateurs de sécurité pour chaque obstacle
        self.t0 = 30 #temps de sécurité en s
        self.r_offset = 5. #offset pour COLREG
        self.psi_offset = np.pi/6 #offset pour COLREG

        self.output = output
        self.opus = op

        self.finished = finished #0 : no shutdown at the end, 1 : shutdown at the end but program running, 2 : shutdown and prgrm ended

    def _odom_callback(self, data):
        t = rospy.get_time()
        x = self.odom[0]
        y = self.odom[1]
        vx = self.odom[2]
        vy = self.odom[3]
        #print(vx,vy,np.sqrt(vx**2+vy**2))
        self.odom[0] = data.pose.pose.position.x
        self.odom[1] = data.pose.pose.position.y
        self.odom[2] = (self.odom[0]-x)/(t-self.odom[6])
        self.odom[3] = (self.odom[1]-y)/(t-self.odom[6])
        self.odom[4] = (self.odom[2]-vx)/(t-self.odom[6])
        self.odom[5] = (self.odom[3]-vy)/(t-self.odom[6])
        self.odom[6] = t

    def _obst_callback(self, data):
        if (self.n_obst == -1) :
            self.n_obst = len(data.states)
            self.dcpa = np.ones(self.n_obst)*sys.float_info.max
            self.time_occur = np.zeros(self.n_obst)
            self.indic1 = np.zeros(self.n_obst)
            self.indic2 = np.zeros(self.n_obst)
            self.obst_states = np.zeros((self.n_obst, 5))
            self.security = np.zeros((self.n_obst,4))
            self.obst_front = np.array(self.n_obst*[True])

        for i in range(self.n_obst) :
            #vx = self.obst_states[i,2]
            #vy = self.obst_states[i,3]
            self.obst_states[i, 0] = data.states[i].x
            self.obst_states[i, 1] = data.states[i].y
            self.obst_states[i, 2] = data.states[i].u*np.cos(data.states[i].psi)
            self.obst_states[i, 3] = data.states[i].u*np.sin(data.states[i].psi)
            front = (np.dot(self.obst_states[i,2:4],self.odom[0:2]-self.obst_states[i,0:2]) > 0) #asv devant obstacle?
            if (front and self.obst_front[i]) :
                self.obst_states[i, 4] += 1
            #self.obst_states[i, 4] = (self.obst_states[i, 2]-vx)/(t-self.obst_states[i, 6])
            #self.obst_states[i, 5] = (self.obst_states[i, 3]-vy)/(t-self.obst_states[i, 6])
            #self.obst_states[i, 6] = t



    def _start_callback(self, data):
        if (not self.start):
            self.start = True
            self.begin_sim = rospy.Time.to_sec(rospy.Time.now())
            self.begin_wall = time.time()
            print("---------------------BEGINNING OF THE SIMULATION---------------------")

    def _finish_callback(self, data) :
        self.security[:,3] = np.sqrt(self.security[:,3]/self.obst_states[:,4])
        f = open(f'{self.output}','a')
        print("---------------------END OF THE SIMULATION---------------------")
        print(f'Duration of the simulation (real time) : {time.time() -self.begin_wall} s')
        print(f'Duration of the simulation (simulation time): {rospy.get_time()-self.begin_sim} s')
        print(f'Number of ships : {self.n_obst}')
        for i in range(self.n_obst) :
            print(f'Ship {i+1}')
            print(f'     --> dCPA = {self.dcpa[i]} m')
            print(f'     --> t = {self.time_occur[i]} s')
            #print(f'     --> indic1 = {self.indic1[i]} m/s')
            #print(f'     --> indic2 = {self.indic2[i]} m/s')
            print(f'     --> security = {self.security[i]}')

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
                for j in range(3):
                    self.security[i, j] = np.maximum(self.security[i, j],secu[j,i])
                if self.obst_front :
                    self.security[i,3] += secu[3,i]**2
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
            dist[i] = np.linalg.norm(self.obst_states[i,:2]-self.odom[:2]) # beware of the map resolution
        return dist

    def ob_secu(self) :
        dist = self.ob_dist()
        rvel = np.zeros(self.n_obst) #relative velocity
        acc = np.linalg.norm(self.odom[4:6]) #asv acceleration
        offd = np.zeros(self.n_obst) #distance avec offset

        #asv_psi = np.arctan2(self.odom[3],self.odom[2])
        #asv_off = self.odom[:2]+rot(self.r_offset*self.odom[2:4]/np.linalg.norm(self.odom[2:4]),self.psi_offset)
        asv_off = self.odom[:2]+rot(self.r_offset*self.odom[2:4]/np.linalg.norm(self.odom[2:4]),-self.psi_offset)


        #print(self.odom[2:4])
        #sim(asv_off,self.psi_offset,self.r_offset)
        #self.odom[:2]+np.array([self.r_offset*np.cos(asv_psi-self.psi_offset),
                                #self.r_offset*np.sin(asv_psi-self.psi_offset)])

        asv_off_marker = Marker()
        asv_off_marker.header.frame_id = "map"
        asv_off_marker.header.stamp    = rospy.get_rostime()
        asv_off_marker.ns = "asv_off_marker"
        asv_off_marker.id = 0
        asv_off_marker.type = 2
        asv_off_marker.action = 0
        asv_off_marker.pose.position.x = asv_off[0]
        asv_off_marker.pose.position.y = asv_off[1]
        asv_off_marker.pose.position.z = 0
        asv_off_marker.pose.orientation.x = 0
        asv_off_marker.pose.orientation.y = 0
        asv_off_marker.pose.orientation.z = 0
        asv_off_marker.pose.orientation.w = 1.0
        asv_off_marker.scale.x = 1.0
        asv_off_marker.scale.y = 1.0
        asv_off_marker.scale.z = 1.0
        asv_off_marker.color.r = 0.5
        asv_off_marker.color.g = 0.0
        asv_off_marker.color.b = 0.5
        asv_off_marker.color.a = 1.0
        asv_off_marker.lifetime = rospy.Duration(0.)
        self.asv_off_publisher.publish(asv_off_marker)

        #obst_psi = np.arctan2(self.obst_states[:,3],self.obst_states[:,2])
        obst_off_marker = Marker()

        for i in range(self.n_obst) :
            rvel[i] = np.linalg.norm(self.obst_states[i,2:4]-self.odom[2:4])            
            #obst_off = self.obst_states[i,:2]+np.array([self.r_offset*np.cos(obst_psi[i]+self.psi_offset),
            #                                            self.r_offset*np.sin(obst_psi[i]+self.psi_offset)])
            obst_off = self.obst_states[i,:2]+rot(self.r_offset*self.obst_states[i,2:4]/np.linalg.norm(self.obst_states[i,2:4]),self.psi_offset)

            obst_off_marker.header.frame_id = "map"
            obst_off_marker.header.stamp    = rospy.get_rostime()
            obst_off_marker.ns = "obst_off_marker"
            obst_off_marker.id = 0
            obst_off_marker.type = 2
            obst_off_marker.action = 0
            obst_off_marker.pose.position.x = obst_off[0]
            obst_off_marker.pose.position.y = obst_off[1]
            obst_off_marker.pose.position.z = 0
            obst_off_marker.pose.orientation.x = 0
            obst_off_marker.pose.orientation.y = 0
            obst_off_marker.pose.orientation.z = 0
            obst_off_marker.pose.orientation.w = 1.0
            obst_off_marker.scale.x = 1.0
            obst_off_marker.scale.y = 1.0
            obst_off_marker.scale.z = 1.0
            obst_off_marker.color.r = 0.5
            obst_off_marker.color.g = 0.0
            obst_off_marker.color.b = 0.5
            obst_off_marker.color.a = 1.0
            obst_off_marker.lifetime = rospy.Duration(0.)
            self.obst_off_publisher.publish(obst_off_marker)
            
            offd[i] = np.linalg.norm(asv_off-obst_off) #beware of the map resolution
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

def rot(u,phi):
    v = np.zeros(2)
    v[0] = np.cos(phi)*u[0]-np.sin(phi)*u[1]
    v[1] = np.sin(phi)*u[0]+np.cos(phi)*u[1]
    return v

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
