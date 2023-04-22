#!/usr/bin/env python3

import socket as s 
import time 
import json 
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import struct
import queue
import threading
import numpy as np

HOST = s.gethostname() 
PORT_PCL = 5001
PORT_ODOM = 5002

conn_pcl = None
conn_odom = None
sender_queue_pcl = queue.Queue()
sender_queue_odom = queue.Queue()


def setupSockets():
    socket_pcl = s.socket(s.AF_INET, s.SOCK_STREAM)
    socket_pcl.bind((HOST, PORT_PCL)) 
    socket_pcl.listen()

    socket_odom = s.socket(s.AF_INET, s.SOCK_STREAM)
    socket_odom.bind((HOST, PORT_ODOM)) 
    socket_odom.listen()

    rospy.loginfo("Listening for Unity connection")
    global conn_pcl, conn_odom
    conn_pcl, _ = socket_pcl.accept()
    conn_odom, _ = socket_odom.accept()





# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
def callback_cloud_map(cloud_map):   
    data = cloud_map.data
    header = struct.pack('!I', len(data))
    message = header + data
    sender_queue_pcl.put(message)
    



# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
def callback_odom(odom):
    
    position = odom.pose.pose.position # x, y, z
    orientation = odom.pose.pose.orientation # quaternion x, y, z, w
    
    data = np.array([position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float32).tobytes()
    header = struct.pack('!I', len(data))
    message = header + data
    sender_queue_odom.put(message)

def senderThreadPcl():
    while True:
        try:
            data = sender_queue_pcl.get(timeout=1)
            conn_pcl.sendall(data)
            sender_queue_pcl.task_done()
        except queue.Empty:
            pass

def senderThreadOdom():
    while True:
        try:
            data = sender_queue_odom.get(timeout=1)
            conn_odom.sendall(data)
            sender_queue_odom.task_done()
        except queue.Empty:
            pass


def main():
    
    # Initialize the node
    rospy.init_node('unity_subsciber', anonymous=True)
    
    # establish connection 
    setupSockets()
    

    rospy.loginfo("Connected to unity")

    # Subscribe to the "/rtabmap/cloud_map" topic
    rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, callback_cloud_map)

    # Subscribe to the "/rtabmap/odom" topic
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_odom)

    # Spin until the node is stopped
    # rospy.spin()

    sender_pcl = threading.Thread(target=senderThreadPcl)
    sender_pcl.daemon = True
    sender_pcl.start()

    sender_odom = threading.Thread(target=senderThreadOdom)
    sender_odom.daemon = True
    sender_odom.start()


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




