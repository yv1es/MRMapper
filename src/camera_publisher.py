#!/usr/bin/env python3
import rospy
import socket as s 
import numpy as np 
from cv_bridge import CvBridge
import cv2
import pickle
import struct
import time

# import ROS messages 
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

from constants import *
fps_counter = 50


def setupSocket():
    socket = s.socket(s.AF_INET, s.SOCK_STREAM)
    socket.bind((HOST, PORT_CAMERA)) 
    socket.listen()
    return socket


def setupCameraInfo():
    # information on parameters. http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    camera_info = CameraInfo()
    camera_info.width = FRAME_WIDTH
    camera_info.height = FRAME_HEIGHT
    camera_info.distortion_model = CAMERA_DISTORTION_MODEL

    camera_info.D = CAMERA_D
    camera_info.K = CAMERA_K                
    camera_info.R = list(np.eye(3).reshape(9).astype(np.float32))
    camera_info.P = list(np.hstack([np.array(CAMERA_K).reshape((3, 3)), np.zeros((3, 1))]).reshape(12).astype(np.float32))
    return camera_info

class Msg():
    def __init__(self, color, depth) -> None:
        self.color = color
        self.depth = depth


def decode(msg_bytes):
    msg = pickle.loads(msg_bytes)
    color = cv2.imdecode(np.frombuffer(msg.color, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth = msg.depth 
    return color, depth


def main():
    # initialize node and topics 
    rospy.init_node('camera_node', anonymous=True)
    color_pub = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)

    # create camera_info and CvBridge
    camera_info = setupCameraInfo()
    bridge = CvBridge()

    rospy.loginfo("[Camera publisher] Waiting for streamer connection")
    socket = setupSocket()
    conn, address = socket.accept()
    rospy.loginfo("Streamer connected")

    start_time = time.time()

    indx = 0 

    # publisher loop 
    while not rospy.is_shutdown():
        
        # Receive the size of the data and then the data itself from the socket connection
        data_size = conn.recv(4)
        size = struct.unpack('!I', data_size)[0]
        data = b''
        while len(data) < size:
            packet = conn.recv(size - len(data))
            if not packet:
                break
            data += packet

        # Convert the byte array to an OpenCV image
        color_image, depth_image = decode(data)
        
        # transform to ROS Image messages
        color_ros = bridge.cv2_to_imgmsg(color_image, encoding="rgb8")
        depth_ros = bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
        

        # set headers 
        current_time = rospy.get_time()
        header = Header(stamp=rospy.Time.from_sec(current_time), frame_id="camera_link")
        color_ros.header = header
        depth_ros.header = header
        camera_info.header = header

        # publish 
        color_pub.publish(color_ros)
        depth_pub.publish(depth_ros)
        info_pub.publish(camera_info)

        
        if indx % fps_counter == 0: 
            elapsed_time = time.time() - start_time
            fps = fps_counter / (elapsed_time)
            # rospy.loginfo(f"FPS: {fps}")
            start_time = time.time()

        indx += 1
        
    conn.close() 
    # rospy.loginfo("Streamer disconnected")



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

