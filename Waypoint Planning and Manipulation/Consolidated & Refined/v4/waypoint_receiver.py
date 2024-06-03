#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def waypoint_callback(data):
    # Extract waypoint data from the received message
    waypoint_x = data.pose.position.x
    waypoint_y = data.pose.position.y
    waypoint_z = data.pose.position.z

    # Extract orientation data from the received message
    orientation_x = data.pose.orientation.x
    orientation_y = data.pose.orientation.y
    orientation_z = data.pose.orientation.z
    orientation_w = data.pose.orientation.w

    # Use the received waypoint data in your team's code
    print("Received waypoint - X: {}, Y: {}, Z: {}".format(waypoint_x, waypoint_y, waypoint_z))
    print("Received orientation - X: {}, Y: {}, Z: {}, W: {}".format(orientation_x, orientation_y, orientation_z, orientation_w))

def main():
    rospy.init_node('waypoint_receiver', anonymous=True)

    # Subscribe to the waypoint_topic
    rospy.Subscriber("waypoint_topic", PoseStamped, waypoint_callback)

    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()
