#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry

def cloud_map_callback(data):
    print("Cloud map")

def odom_callback(data):
    print("Odom")

def image_callback(data):
    print("Image")

def main():
    rospy.init_node('my_node_name', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
