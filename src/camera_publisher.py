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

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60
HOST = s.gethostname() 
PORT = 5000 

FPS_COUNTER = 50

def setupSocket():
    socket = s.socket(s.AF_INET, s.SOCK_STREAM)
    socket.bind((HOST, PORT)) 
    socket.listen()
    return socket


def setupCameraInfo():
    # information on parameters. http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    camera_info = CameraInfo()
    camera_info.width = FRAME_WIDTH
    camera_info.height = FRAME_HEIGHT
    camera_info.distortion_model = "plumb_bob"

    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]


    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    camera_info.K = [618.3,   0.0, 316.2,
                       0.0, 617.9, 242.3,
                       0.0,   0.0,   1.0]                   
    
    camera_info.R = [1.0, 0.0, 0.0, 
                     0.0, 1.0, 0.0, 
                     0.0, 0.0, 1.0]

    camera_info.P = [618.3,   0.0, 316.2, 0.0,
                       0.0, 617.9, 242.3, 0.0,
                       0.0,   0.0,   1.0, 0.0] 
    
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

    rospy.loginfo("Waiting for streamer connection")
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
        # color_ros = cv2_to_imgmsg(color_image, encoding="rgb8")
        # depth_ros = cv2_to_imgmsg(depth_image, encoding="mono16")
        

        

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

        
        if indx % FPS_COUNTER == 0: 
            elapsed_time = time.time() - start_time
            fps = FPS_COUNTER / (elapsed_time)
            rospy.loginfo(f"FPS: {fps}")
            start_time = time.time()

        indx += 1
        
    conn.close() 
    rospy.loginfo("Streamer disconnected")



import sys
import numpy as np
from sensor_msgs.msg import Image

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image, encoding='bgr8'):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

