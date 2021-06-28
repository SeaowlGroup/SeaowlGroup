#!/usr/bin/env python3

import numpy as np
import rospy
import geometry_msgs.msg
import nav_msgs.msg
from visualization_msgs.msg import Marker
from asv_msgs.msg import Path
from std_msgs.msg import Empty
from utils import Controller

class LOSGuidanceROS(object):
    """A ROS wrapper for LOSGuidance()"""
    def __init__(self,
                 R2=20**2, #Distance of security
                 u_d=2.0,
                 de=50.0,
                 Ki=0.0,
                 dt=0.2, #Parameter of time
                 max_integral_correction=np.pi*20.0/180.0,
                 switch_criterion='circle'):

        self.controller = LOSGuidance(R2,
                                      u_d,
                                      de,
                                      Ki,
                                      dt,
                                      max_integral_correction,
                                      switch_criterion)
        self.rate = dt
        self.wp   = self.controller.wp
        self.nwp  = 0 #number of waypoints
        self.cwp  = 0 #current waypoint

        self._cmd_publisher   = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._odom_subscriber = rospy.Subscriber("state", nav_msgs.msg.Odometry, self._odom_callback, queue_size=1)

        self.odom = nav_msgs.msg.Odometry()
        self.cmd  = geometry_msgs.msg.Twist()
        self.cmd.linear.x = u_d

    def set_waypoints(self, wps):
        self.wp = wps
        self.controller.wp = np.copy(wps)
        self.controller.nWP = len(wps)
        self.controller.wp_initialized = True
        self.nwp = len(wps)


    def _odom_callback(self, data):
        self.odom = data

    def _update(self):

        u_d, psi_d, switched = self.controller.update(self.odom.pose.pose.position.x,
                                                      self.odom.pose.pose.position.y)
        if switched:
            #print "Switched!"
            self.cwp += 1

        # Publish cmd_vel
        self.cmd.linear.x = u_d
        self.cmd.angular.y = psi_d
        self.cmd.angular.z = 0.0

        self._cmd_publisher.publish(self.cmd)

    def run_controller(self):
        r = rospy.Rate(1/self.rate)

        while not rospy.is_shutdown():
            self._update()
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.is_shutdown():
                    break
                raise

class LOSGuidance(Controller):
    """This class implements the classic LOS guidance scheme."""
    def __init__(self,
                 R2=20**2,
                 u_d=2.0,
                 de=50.0,
                 Ki=0.0,
                 dt=0.2,
                 max_integral_correction=np.pi*20.0/180.0,
                 switch_criterion='circle'):
        self.R2 = R2 # Radii of acceptance (squared)
        self.R  = np.sqrt(R2)
        self.de = de # Lookahead distance

        self.dt = dt
        self.max_integral_correction = np.abs(np.tan(max_integral_correction) * de)
        self.Ki = Ki

        self.e_integral = 0.0

        self.cWP = 0 # Current waypoint
        self.wp = None
        self.nWP = 0
        self.wp_initialized = False

        if switch_criterion == 'circle':
            self.switching_criterion = self.circle_of_acceptance
        elif switch_criterion == 'progress':
            self.switching_criterion = self.progress_along_path

        self.Xp = 0.0
        self.u_d = u_d


    def __str__(self):
        return """Radii: %f\nLookahead distance: %f\nCurrent Waypoint: %d"""%(self.R, self.de, self.cWP)

    def circle_of_acceptance(self, x, y):
        return \
            (x - self.wp[self.cWP][0])**2 + \
            (y - self.wp[self.cWP][1])**2 < self.R2

    def progress_along_path(self, x, y):
        return \
            np.abs((self.wp[self.cWP][0] - x)*np.cos(self.Xp) + \
                   (self.wp[self.cWP][1] - y)*np.sin(self.Xp)) < self.R

    def update(self, x, y):
        if not self.wp_initialized:
            #print "Error. No waypoints!"
            return 0,0,False

        if self.R2 > 999999:
            # Last waypoint has been reached.
            return 0, self.Xp, False

        #print self.wp[self.cWP,:], str(self)
        switched = False

        if self.switching_criterion(x, y):
            while self.switching_criterion(x,y):
                if self.cWP < self.nWP - 1:
                # There are still waypoints left
                    #print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                    #                                              self.wp[self.cWP][0],
                    #                                              self.wp[self.cWP][1])
                    new_course = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                            self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])

                    if (np.abs(new_course - self.Xp) > np.pi/4.0):
                        self.e_integral = 0.0

                    self.Xp = new_course
                    self.cWP += 1
                    switched = True
                else:
                    # Last waypoint reached

                    if self.R2 < 50000:
                        #print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                        #                                              self.wp[self.cWP][0],
                        #                                              self.wp[self.cWP][1])
                        #print "Last Waypoint reached!"
                        self.R2 = np.Inf
                    return 0, self.Xp, False

        xk = self.wp[self.cWP][0]
        yk = self.wp[self.cWP][1]

        # Cross-track error Eq. (10.10), [Fossen, 2011]
        e  = -(x - xk)*np.sin(self.Xp) + (y - yk)*np.cos(self.Xp)
        self.e_integral += e*self.dt

        if self.e_integral*self.Ki > self.max_integral_correction:
            self.e_integral -= e*self.dt

        Xr = np.arctan2( -(e + self.Ki*self.e_integral), self.de)

        psi_d = self.Xp + Xr

        return self.u_d, psi_d, switched
def dist_init(u_d, u_d_asv, theta, t_col) :
    return (u_d_asv*t_col)**2 + (u_d*t_col)**2 - 2*u_d*u_d_asv*(t_col**2)*np.cos(theta)

def delay(u_d, u_d_asv, theta, t_col) :
    return (u_d_asv*t_col)**2 + (u_d*t_col)**2 - 2*u_d*u_d_asv*(t_col**2)*np.cos(theta)

if __name__ == "__main__":
    rospy.init_node("obstacles_controller")

    prior = rospy.get_param("~prior", [])
    size = rospy.get_param("~size", [])
    heading = rospy.get_param("~heading", [])
    u_d = rospy.get_param("~u_d", [])
    t_collision = rospy.get_param("~t_collision", [])
    d_detection = rospy.get_param("~d_detection", [])
    dcpa = rospy.get_param("~dcpa", [])

    initial_state_asv = rospy.get_param("~initial_state_asv")

    dt = rospy.get_param("~update_rate", .2)

    #cmd_publisher   = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
    #pos_pub = rospy.Publisher("state", nav_msgs.msg.Odometry, queue_size=1)
    states_pub = rospy.Publisher("/obstacle_states", StateArray, queue_size=1)

    N = len(u_d)
    calc_heading = (90-np.array(heading))*np.pi/180
    delay_time = np.zeros(N)
    for i in range(N) :
        # Time of apparition
        theta = initial_state_asv[2] - calc_heading[i]
        if dist_init(u_d[i], initial_state_asv[3], theta, t_collision[i]) > d_detection[i] :
            delay_time[i] = (t_collision[i] -
                          d_detection[i]/np.sqrt(u_d[i]**2 +
                                          initial_state_asv[3]**2 -
                                          2*initial_state_asv[3]*u_d*np.cos(theta)))
        # Trajectory
        or_x = 0.0
        or_y = 0.0
        first_point_x = (t_collision*u_d_asv*np.cos(calc_heading_asv) -
                        (t_collision-delay_time)*u_d*np.cos(calc_heading) +
                        or_x + dcpa*np.cos(calc_heading_asv))
        first_point_y = (t_collision*u_d_asv*np.sin(calc_heading_asv) -
                        (t_collision-delay_time)*u_d*np.sin(calc_heading) +
                        or_y + dcpa*np.sin(calc_heading_asv))
        last_point_x = first_point_x + 3*t_collision*u_d*np.cos(calc_heading)
        last_point_y = first_point_y + 3*t_collision*u_d*np.sin(calc_heading)

        initial_state = [first_point_x, first_point_y, calc_heading, u_d, 0., 0.]
        waypoints = [[first_point_x, first_point_y], [last_point_x, last_point_y]]
        #shipname = type+f'{i+1}'
        shipname = 'ship1'






    guide.run_controller()
