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


# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
def callback_cloud_map(cloud_map):   
    data = cloud_map.data
    sender_pcl.send(data)
    

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
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



class UnitySender:
    def __init__(self, ip, port, tag) -> None:        
        self.ip = ip 
        self.port = port 
        self.tag = tag
        self.sender_queue = queue.Queue()
        self.running = True
        self.conn = None
        

    def start(self):
        self.socket = s.socket(s.AF_INET, s.SOCK_STREAM)
        self.socket.bind((self.ip, self.port)) 
        self.socket.listen()
        self.log("Listening incoming unity connection")
        self.conn, _ = self.socket.accept()
        self.log("Connected to unity")

        self.sender = threading.Thread(target=self.senderThread)
        self.sender.daemon = True
        self.sender.start()
        self.log("Sender thread running")


    def log(self, message):
        rospy.loginfo("[{}] {}".format(self.tag, message))


    def senderThread(self):
        while self.running:
                try:
                    data = self.sender_queue.get(timeout=1)
                    self.conn.sendall(data)
                    self.sender_queue.task_done()
                except queue.Empty:
                    pass


    def send(self, data):
        header = struct.pack('!I', len(data))
        message = header + data
        self.sender_queue.put(message)


    def stop(self):
        self.running = False
        self.conn.close()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        sender_pcl.stop()
        sender_odom.stop()




