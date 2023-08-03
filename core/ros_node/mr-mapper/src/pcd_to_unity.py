#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2

from unity_sender import UnitySender 
import constants
"""
This node sends the point cloud published by rtabmap_ros to unity 
"""


def callback_cloud_map(cloud_map):   
    data = cloud_map.data
    sender_pcd.send(data)
    
def main():    
    rospy.init_node('pcd_to_unity', anonymous=True)

    # Create UnitySender for point cloud data
    global sender_pcd, sender_odom
    sender_pcd = UnitySender(constants.HOST, constants.PORT_PCL, 'Pcd Sender')
    sender_pcd.start()

    rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, callback_cloud_map)
    rospy.on_shutdown(shutdown)
    rospy.spin()
    

def shutdown():
    sender_pcd.stop()
    print("pcd_to_unity shutdown")

if __name__ == '__main__':
    try:
        main()
    finally:
        shutdown()




