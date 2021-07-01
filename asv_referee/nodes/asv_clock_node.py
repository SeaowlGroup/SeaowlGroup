#!/usr/bin/env python3

import rospy
import time
from rosgraph_msgs.msg import Clock

if __name__ == '__main__':
    pub = rospy.Publisher('/clock',Clock, queue_size=1)
    rospy.init_node('simClock')
    t = rospy.Time()
    t0 = time.time()
    speed = 100
    while not rospy.is_shutdown():
        t = rospy.Time.from_sec(speed*(time.time()-t0))
        pub.publish(t)
        time.sleep(1/(2*speed))

# if __name__ == '__main__':
#     pub = rospy.Publisher('/clock',Clock, queue_size=10)
#     rospy.init_node('talker')
#     rate = rospy.Rate(10) # 10hz
#     sim_speed_multiplier = 10
#     sim_clock = Clock()
#     #zero_time = rospy.get_time()
#     simtime = 0
#     try:
#         while not rospy.is_shutdown():
#            sim_clock.clock = rospy.Time.from_sec(sim_speed_multiplier*(simtime))
#            pub.publish(sim_clock)
#            simtime += 0.1
#            #rate.sleep()
#            time.sleep(0.01)
#     except rospy.ROSInterruptException:
#         pass
# '''
# import rospy
# from rosgraph_msgs.msg import Clock
# from std_msgs.msg import String
#
# rospy.init_node('tmp')
# pub = rospy.Publisher('/clock', Clock, queue_size=10)
# print "node initialized"
#
# while not rospy.is_shutdown():
#     t = rospy.Time.now()
#     msg = Clock()
#     msg.clock = t
#     print t.to_sec()
#     pub.publish(msg)
# '''
