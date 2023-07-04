#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry

from unity_sender import UnitySender 
from constants import *

""" 
This node sends the current camera position and orientation from rtabmap_ros to unity. 
"""


def callback_odom(odom):
    position = odom.pose.pose.position 
    orientation = odom.pose.pose.orientation 
    data = np.array([position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float32).tobytes()
    sender_odom.send(data)
    

def main():    
    rospy.init_node('odom_to_unity', anonymous=True)
    
    # create UnitySender for odometry data
    global sender_odom
    sender_odom = UnitySender(HOST, PORT_ODOM, 'Odom Sender')
    sender_odom.start()

    rospy.Subscriber("/rtabmap/odom", Odometry, callback_odom)
    rospy.on_shutdown(shutdown)
    rospy.spin()


def shutdown():
    sender_odom.stop()
    print("sense_making shutdown")


if __name__ == '__main__':
    try:
        main()
    finally:
        shutdown()




