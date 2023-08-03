# !/usr/bin/env python3
from sensor_msgs.point_cloud2 import read_points
import struct
import open3d as o3d
import numpy as np
import tf

from unity_sender import UnitySender 
from constants import *


# infinity 
INF = float('inf') 


def transform_matrix_from_odom(odom_msg):
    """
    Converts the pose information from an odometry message to a transformation matrix.

    Args:
        odom_msg (Odometry): The odometry message containing the pose information.

    Returns:
        ndarray: The transformation matrix.
    """
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation    
    euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    T = tf.transformations.compose_matrix(translate=[position.x, position.y, position.z], angles=euler)
    return T


def float_to_color(f):
    """
    Converts a float value to RGB color representation.

    Args:
        f (float): The float value.

    Returns:
        tuple: The RGB color tuple (r, g, b).
    """
    # Convert the float to bytes
    b = struct.pack('f', f)

    # Extract the red, green, and blue components from the first three bytes
    r, g, b = struct.unpack('BBB', b[:3])
    return r, g, b


def obox_to_corners(obb):
    """
    Converts an OrientedBoundingBox to a list of corner points.

    Args:
        obb (OrientedBoundingBox): The OrientedBoundingBox object.

    Returns:
        ndarray: The corner points as a NumPy array.
    """
    plane = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obb, scale=[1, 1, 1e-30])
    points = np.asarray(plane.vertices)
    points = np.unique(points, axis=0)
    return points


def load_chair_mesh():
    """
    Loads the chair mesh from a file.

    Returns:
        TriangleMesh: The loaded chair mesh.
    """
    chair_mesh = o3d.io.read_triangle_mesh(CHAIR_MESH_PATH)
    chair_mesh.compute_vertex_normals()
    chair_mesh.paint_uniform_color([0, 0, 1]) 
    return chair_mesh

def ros_pointcloud_to_o3d(pointcloud_msg):
    """
    Converts a ROS PointCloud2 message to an Open3D PointCloud object.

    Args:
        pointcloud_msg (PointCloud2): The ROS PointCloud2 message.

    Returns:
        PointCloud: The converted Open3D PointCloud object.
    """

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
    colors = np.column_stack((b_vals, g_vals, r_vals))

    # Create an Open3D point cloud from the points and colors arrays
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(points)
    pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors / 255.0)

    return pointcloud_o3d

class Msg():
    def __init__(self, color, depth) -> None:
        self.color = color
        self.depth = depth