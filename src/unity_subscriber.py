#!/usr/bin/env python3

import socket as s 
import time 
import json 
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


HOST = s.gethostname() 
PORT = 5001

conn = None
connected = False

def setupSocket():
    socket = s.socket(s.AF_INET, s.SOCK_STREAM)
    socket.bind((HOST, PORT)) 
    socket.listen()
    return socket


def sendToUnity(msg):
    # if not connected: 
    #     return 
    print("SENDING")
    json_data = json.dumps(msg)
    
    
    conn.sendall(json_data.encode())



# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
def callback_cloud_map(cloud_map):
    pass


# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
def callback_odom(odom):
    
    position = odom.pose.pose.position # x, y, z
    orientation = odom.pose.pose.orientation # quaternion x, y, z, w
    msg_dict = {
        'position':[position.x, position.y, position.z],
        'orientation':[orientation.x, orientation.y, orientation.z, orientation.w]
    }
    sendToUnity(msg_dict)

    
def main():
    
    # Initialize the node
    rospy.init_node('unity_subsciber', anonymous=True)
    
    global conn
   
    while True:
        try:
            socket = setupSocket()
            rospy.loginfo("Listening for Unity connection")
            conn, address = socket.accept()
            connected = True
            break
        except:
            rospy.loginfo("Error Unity connection")
            time.sleep(1)

    rospy.loginfo("Connected to unity")


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
