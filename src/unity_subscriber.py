#!/usr/bin/env python3

import socket as s 

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


HOST = s.gethostname() 
PORT = 5000


# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
def callback_cloud_map(cloud_map):
    # print(cloud_map)
    pass

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
def callback_odom(odom):
    position = odom.pose.pose.position # x, y, z
    orientation = odom.pose.pose.orientation # quaternion x, y, z, w
    

    
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
