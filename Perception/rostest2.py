import rospy
from std_msgs.msg import Int64MultiArray
import time

def publish_incrementing_array():
    rospy.init_node('incrementing_array_publisher', anonymous=True)
    pub = rospy.Publisher('incrementing_array', Int64MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    array_msg = Int64MultiArray()

    # Initialize array
    increment = 1
    array_data = [0, 0, 0]  # Example array with three elements
    array_msg.data = array_data

    while not rospy.is_shutdown():
        # Increment each element in the array
        array_data = [data + increment for data in array_data]
        array_msg.data = array_data

        pub.publish(array_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_incrementing_array()
    except rospy.ROSInterruptException:
        pass