#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

def callback_cloud_map(data):
    # Callback function for the "/rtabmap/cloud_map" topic
    rospy.loginfo(rospy.get_caller_id() + "I heard a cloud map")

def callback_odom(data):
    # Callback function for the "/rtabmap/odom" topic
    rospy.loginfo(rospy.get_caller_id() + "I heard an odom message")

def main():
    # Initialize the node
    rospy.init_node('unity_subsciber', anonymous=True)

    # Subscribe to the "/rtabmap/cloud_map" topic
    rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, callback_cloud_map)

    # Subscribe to the "/rtabmap/odom" topic
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_odom)

    # Spin until the node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
