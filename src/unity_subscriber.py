#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from unity_sender import UnitySender 
from constants import *


def callback_cloud_map(cloud_map):   
    data = cloud_map.data
    sender_pcl.send(data)
    

def callback_odom(odom):
    position = odom.pose.pose.position 
    orientation = odom.pose.pose.orientation 
    data = np.array([position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float32).tobytes()
    sender_odom.send(data)
    

def main():    
    rospy.init_node('unity_subsciber', anonymous=True)
    
    global sender_pcl, sender_odom
    sender_pcl = UnitySender(HOST, PORT_PCL, 'Pcl Sender')
    sender_odom = UnitySender(HOST, PORT_ODOM, 'Odom Sender')

    sender_pcl.start()
    sender_odom.start()

    rospy.loginfo("Connected to unity")
    
    rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, callback_cloud_map)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_odom)

    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        sender_pcl.stop()
        sender_odom.stop()




