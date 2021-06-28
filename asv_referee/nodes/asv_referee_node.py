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
                 output='/home/soubi/Documents/SEAOWL/nonor_ws/src/ros_asv_system/asv_system/output/test',
                 op='0') :

        self.begin_wall = 0.
        self.begin_sim = 0.
        self.start = False
        self.rate = dt

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
                                                    queue_size=1)
        self.start_publisher   = rospy.Publisher("/start_simultaion", Empty, queue_size=1)

        self.odom = np.zeros(4)
        self.n_obst = -1
        self.obst_states = [];
        self.dcpa = []
        self.time_occur = []
        self.indic1 = []
        self.indic2 = []

        self.output = output
        self.opus = op

        self.finished = finished #0 : no shutdown at the end, 1 : shutdown at the end but program running, 2 : shutdown and prgrm ended

    def _odom_callback(self, data):
        self.odom = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.twist.twist.linear.x, data.twist.twist.linear.y])

    def _obst_callback(self, data):
        if (self.n_obst == -1) :
            self.n_obst = len(data.states)
            self.dcpa = np.ones(self.n_obst)*sys.float_info.max
            self.time_occur = np.zeros(self.n_obst)
            self.indic1 = np.zeros(self.n_obst)
            self.indic2 = np.zeros(self.n_obst)
            self.obst_states = np.zeros((self.n_obst, 4))
        for i in range(self.n_obst) :
            self.obst_states[i, 0] = data.states[i].x
            self.obst_states[i, 1] = data.states[i].y
            self.obst_states[i, 2] = data.states[i].u
            self.obst_states[i, 3] = data.states[i].psi

    def _start_callback(self, data):
        if (not self.start):
            self.begin_sim = rospy.Time.to_sec(rospy.Time.now())
            self.begin_wall = time.time()
            print("---------------------BEGINNING OF THE SIMULATION---------------------")

    def _finish_callback(self, data) :
        f = open(f'{self.output}','a')
        print("---------------------END OF THE SIMULATION---------------------")
        print(f'Duration of the simulation (real time) : {rospy.get_time() -self.begin_wall} s')
        print(f'Duration of the simulation (simulation time): {rospy.Time.to_sec(rospy.Time.now())-self.begin_sim} s')
        print(f'Number of ships : {self.n_obst}')
        for i in range(self.n_obst) :
            print(f'Ship {i+1}')
            print(f'     --> dCPA = {self.dcpa[i]} m')
            print(f'     --> t = {self.time_occur[i]} s')
            print(f'     --> indic1 = {self.indic1[i]} m/s')
            print(f'     --> indic2 = {self.indic2[i]} m/s')
            f.write(f'OPUS {self.opus} : {np.min(self.dcpa)}\n')
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
            for i in range(self.n_obst) :
                if (d[i] < self.dcpa[i]) :
                    self.dcpa[i] = d[i]
                    #norm_ot = np.array([self.odom[0] - self.obst_states[i, 0], self.odom[1] - self.obst_states[i, 1]])
                    norm_ot = self.obst_states[i,:2] - self.odom[:2]
                    norm_ot /= np.linalg.norm(norm_ot)
                    aux = np.array([self.obst_states[i, 2]*np.cos(self.obst_states[i, 3]), self.obst_states[i, 2]*np.sin(self.obst_states[i, 3])])
                    aux2 = np.array([self.odom[2], self.odom[3]])
                    self.indic1[i] = - np.dot(aux, norm_ot)
                    self.indic2[i] = np.dot(norm_ot, aux2)
                    self.time_occur[i] = rospy.get_time() - self.begin_sim
        #self.sim_time += self.rate

    def ob_dist(self) :
        dist = np.zeros(self.n_obst)
        for i in range(self.n_obst) :
            dist[i] = np.linalg.norm(self.obst_states[i]-self.odom) # beware of the map resolution
        return dist

    def run_controller(self):
        r = rospy.Rate(1/self.rate)

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

    dt = rospy.get_param("~update_rate", .2)
    finished = rospy.get_param("~shutdown", 0)
    output = rospy.get_param("~output_file", '/home/soubi/Documents/SEAOWL/nonor_ws/src/ros_asv_system/asv_system/output/test.txt')
    op = rospy.get_param("~opus", '0')

    print('Output : ', output)

    ref = Referee(dt, finished, output, op)

    #top départ (temporary)
    ref.start_publisher.publish(Empty)

    ref.run_controller()
