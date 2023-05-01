#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.point_cloud2 import read_points

import struct
import open3d as o3d
import numpy as np
import os
import datetime
import cv2

capture = False
cloud_map_msg= None
odom_msg = None
image_msg = None

def cloud_map_callback(data):
    global cloud_map_msg
    if capture and cloud_map_msg is None:
        cloud_map_msg = data

def odom_callback(data):
    global odom_msg
    if capture and odom_msg is None:
        odom_msg = data

def image_callback(data):
    global image_msg
    if capture and image_msg is None:
        image_msg = data





def float_to_color(f):
    # Convert the float to bytes
    b = struct.pack('f', f)

    # Extract the red, green, and blue components from the first three bytes
    r, g, b = struct.unpack('BBB', b[:3])

    return r, g, b

def create_o3d_pointcloud(x, y, z, r, g, b):
    # Concatenate the X, Y, and Z arrays into a single array
    points = np.column_stack((x, y, z))

    # Concatenate the R, G, and B arrays into a single array
    colors = np.column_stack((r, g, b))

    # Create an Open3D point cloud from the points and colors arrays
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(points)
    pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors / 255.0)

    return pointcloud_o3d

def ros_pointcloud_to_o3d(pointcloud_msg):

    # Get the point cloud data as a list of tuples (x, y, z)
    pointcloud_data = read_points(pointcloud_msg, skip_nans=True)

    x_vals = []
    y_vals = []
    z_vals = []
    r_vals = []
    g_vals = []
    b_vals = []

    # Iterate over the (x, y, z, c) values and append to the lists
    for p in pointcloud_data:
        x, y, z, c = p
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        r, g, b = float_to_color(c)
        r_vals.append(r)
        g_vals.append(g)
        b_vals.append(b)

    # Convert the lists to numpy arrays
    x_vals = np.array(x_vals)
    y_vals = np.array(y_vals)
    z_vals = np.array(z_vals)
    r_vals = np.array(r_vals)
    g_vals = np.array(g_vals)
    b_vals = np.array(b_vals)

    # Concatenate the X, Y, and Z arrays into a single array
    points = np.column_stack((x_vals, y_vals, z_vals))

    # Concatenate the R, G, and B arrays into a single array
    colors = np.column_stack((r_vals, g_vals, b_vals))

    # Create an Open3D point cloud from the points and colors arrays
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(points)
    pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors / 255.0)

    return pointcloud_o3d
    



def main():
    global capture, cloud_map_msg, odom_msg, image_msg
    
    rospy.init_node('capture_node', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)
    
    capture = False
    cloud_map_msg= None
    odom_msg = None
    image_msg = None
    
    capture = True
    while cloud_map_msg is None or odom_msg is None or image_msg is None: 
        continue # wait 

    rospy.loginfo("Captured triplet")

    # convert to open3d point cloud 
    pcd = ros_pointcloud_to_o3d(cloud_map_msg)

    # Create the point_cloud_data directory in the home directory if it doesn't exist
    home_dir = os.path.expanduser("~")
    data_dir = os.path.join(home_dir, "point_cloud_data")
    if not os.path.exists(data_dir):
        os.mkdir(data_dir)

    # Get the current date and time
    now = datetime.datetime.now()
    date_str = now.strftime("%Y-%m-%d_%H-%M-%S")


    # Construct the filename for the point cloud
    filename_pcl = f"point_cloud_{date_str}.ply"
    filepath_pcl = os.path.join(data_dir, filename_pcl)

    # Save the point cloud to the file
    o3d.io.write_point_cloud(filepath_pcl, pcd)


    # Construct the filename for the image
    filename_img = f"image_{date_str}.png"
    filepath_img = os.path.join(data_dir, filename_img)

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    cv2.imwrite(filepath_img, img)


    rospy.loginfo("Saved to file")
    
    # rospy.spin()

if __name__ == '__main__':
    main()
