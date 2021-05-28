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

        self.odom = np.zeros(2)
        self.n_obst = -1
        self.obst_states = [];
        self.dcpa = []

        self.finished = finished #0 : no shutdown at the end, 1 : shutdown at the end but program running, 2 : shutdown and prgrm ended

    def _odom_callback(self, data):
        self.odom = np.array([data.pose.pose.position.x, data.pose.pose.position.y])

    def _obst_callback(self, data):
        if (self.n_obst == -1) :
            self.n_obst = len(data.states)
            self.dcpa = np.ones(self.n_obst)*sys.float_info.max
            self.obst_states = np.zeros((self.n_obst, 2))
        for i in range(self.n_obst) :
            self.obst_states[i, 0] = data.states[i].x
            self.obst_states[i, 1] = data.states[i].y

    def _finish_callback(self, data) :
        print "---------------------END OF THE SIMULATION---------------------"
        print 'Duration of the simulation : %.2f s' % (rospy.get_time() -self.begin)
        print 'Number of ships : %d' % (self.n_obst)
        for i in range(self.n_obst) :
            print 'Ship %d --> dCPA = %.2f m' % (i+1, self.dcpa[i])
        print "---------------------------------------------------------------"
        #write in an extern file
        if self.finished == 1 :
            self.finished = 2

    def _update(self):
        if (self.n_obst > -1) :
            self.dcpa = np.minimum(self.dcpa, self.ob_dist())
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
            r.sleep()
        rospy.on_shutdown()

if __name__ == "__main__" :

    rospy.init_node("Referee")

    dt = rospy.get_param("~update_rate", .2)
    finished = rospy.get_param("~shutdown", 0)

    ref = Referee(dt, finished)

    ref.run_controller()
