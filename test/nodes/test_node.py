import rospy

if __name__ == "__main__" :
    print("main")
    rospy.init_node("test_node")
    print("init")
    op = rospy.get_param("opus",-1)
    try:
        while not rospy.is_shutdown():
            print(f'op : {op}')
            rospy.signal_shutdown(f'end {op}')
    except rospy.ROSInterruptException:
        pass