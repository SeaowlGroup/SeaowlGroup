#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from asv_msgs.msg import StateArray

import sys

class Referee(object) :

    def __init__(self, dt=0.2, finished=0) :

        self.begin = rospy.get_time()
        self.sim_time = 0
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

        self.odom = np.zeros(4)
        self.n_obst = -1
        self.obst_states = [];
        self.dcpa = []
        self.time_occur = []
        self.indic1 = []
        self.indic2 = []

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


    def _finish_callback(self, data) :
        print "---------------------END OF THE SIMULATION---------------------"
        print 'Duration of the simulation (real time) : %.2f s' % (rospy.get_time() -self.begin)
        print 'Duration of the simulation (simulation time): %.2f s' % (self.sim_time)
        print 'Number of ships : %d' % (self.n_obst)
        for i in range(self.n_obst) :
            print 'Ship %d ' % (i+1)
            print '     --> dCPA = %.2f m' % (self.dcpa[i])
            print '     --> t = %.2f ' % (self.time_occur[i])
            print '     --> indic1 = %.2f m/s' % (self.indic1[i])
            print '     --> indic2 = %.2f m/s' % (self.indic2[i])
        print "---------------------------------------------------------------"
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
                    self.time_occur[i] = rospy.get_time() - self.begin
        self.sim_time += self.rate

    def ob_dist(self) :
        dist = np.zeros(self.n_obst)
        for i in range(self.n_obst) :
            dist[i] = np.linalg.norm(self.obst_states[i]-self.odom)
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

    ref = Referee(dt, finished)

    ref.run_controller()
