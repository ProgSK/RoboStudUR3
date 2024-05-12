import rospy
from std_msgs.msg import Int32

def counter_publisher():
    rospy.init_node('counter_publisher', anonymous=True)
    pub = rospy.Publisher('counter', Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    counter = 0

    while not rospy.is_shutdown():
        counter += 1
        rospy.loginfo("Publishing counter: %d" % counter)
        pub.publish(counter)
        rate.sleep()

if __name__ == '__main__':
    try:
        counter_publisher()
    except rospy.ROSInterruptException:
        pass