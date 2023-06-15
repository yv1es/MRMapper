# !/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.point_cloud2 import read_points
import struct
import open3d as o3d
import numpy as np
import copy 

from unity_sender import UnitySender 
from constants import *

frustum_depth = 10

def pcd_from_bbox(box, extrinsics, pcd):
    intrinsics = np.array(CAMERA_K).reshape((3, 3))

    x1, y1, x2, y2 = box
    
    object_corners =  np.array([
        [x1, y1],
        [x2, y1],
        [x2, y2],
        [x1, y2]
    ])

    # compute mesh of frustum goint through the object corners
    object_frustum = frustum_mesh_from_image_coords(object_corners, 10, intrinsics, extrinsics, FRAME_WIDTH, FRAME_HEIGHT)

    # add mesh for SDF computation 
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_frustum)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh) 

    # compute the SDF of the frustum to make inside outside check 
    signed_distance = scene.compute_signed_distance(np.array(pcd.points).astype(np.float32))
    sd = signed_distance.numpy()
    pcd_bbox = pcd.select_by_index(np.where(sd <= 0)[0])
    return pcd_bbox



# origin is relative to the camera hence normally 0's 
# the corners are also relative to the camera 
def frustum_from_corners(origin, corners, extrinsics):
    mesh = o3d.geometry.TriangleMesh()

    faces = np.array([[0, 2, 1], [0, 3, 2], [0, 4, 3], [0, 1, 4], [1, 2, 3], [3, 4, 1]])

    mesh.vertices = o3d.utility.Vector3dVector(np.vstack((origin, corners)))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(faces))
    mesh.transform(extrinsics)
    
    return mesh


# takes 2D points from the image and gives a 3D mesh of the frustum projected in 3d 
def frustum_mesh_from_image_coords(points, frustum_depth, intrinsics, extrinsics, width, height):
    vecs = image_points_to_direction(points, intrinsics, extrinsics, width, height) 
    vecs *= frustum_depth
    mesh = frustum_from_corners(np.zeros(3), vecs, extrinsics)
    return mesh


# computes the direction unit vectors pointing from the camera to the points
def image_points_to_direction(points, intrinsics, extrinsics, width, height):
    fx, fy = intrinsics[0,0], intrinsics[1,1]
    cx, cy = intrinsics[0,2], intrinsics[1,2]
    
    p = points.astype(float).copy()
    
    p[:,0] = (p[:,0] - cx) / fx
    p[:,1] = (p[:,1] - cy) / fy 

    vectors = np.hstack([-np.ones((p.shape[0], 1)), p])
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    vectors = -vectors / norms

    return vectors



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
    colors = np.column_stack((b_vals, g_vals, r_vals))

    # Create an Open3D point cloud from the points and colors arrays
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(points)
    pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors / 255.0)

    return pointcloud_o3d


def float_to_color(f):
    # Convert the float to bytes
    b = struct.pack('f', f)

    # Extract the red, green, and blue components from the first three bytes
    r, g, b = struct.unpack('BBB', b[:3])
    return r, g, b


def obox_to_corners(obb):
    plane = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obb, scale=[1, 1, 1e-30])
    points = np.asarray(plane.vertices)
    points = np.unique(points, axis=0)
    return points


def load_chair_mesh():
    chair_size = 1.30 # size of the chair in meters along its longes extend 
    chair_mesh = o3d.io.read_triangle_mesh("office_chair.stl")
    chair_points = chair_mesh.vertices
    max_extent = np.amax(chair_points, axis=0)
    min_extent = np.amin(chair_points, axis=0)
    size = max_extent - min_extent
    scaling =  chair_size / np.max(size)
    chair_mesh.paint_uniform_color([0, 0, 1]) 
    chair_mesh.compute_vertex_normals()
    chair_mesh.scale(scaling, center=chair_mesh.get_center())


def icp_fit_object(obj_pcd, frustum_pcd, camera_pos):

    frustum_mean = np.mean(np.asarray(frustum_pcd.points), axis=0)
    initial_pos = (frustum_mean - camera_pos) * 0.5  
    translation = np.eye(4)
    translation[0:3, 3] = initial_pos - obj_pcd.get_center()
   
    # 45 degree homogeneus rotation matrix
    y_45deg = np.array([[0.707, -0.707, 0, 0],
                        [0.707, 0.707, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    
    avg_rmse = 0
    best_rmse = 10e10
    best_t = None

    # skip through possible initial rotations with 45 deg stride
    for k in range(8):
        t = translation @ np.linalg.matrix_power(y_45deg, k) 
        t, rmse = iterative_icp(obj_pcd, frustum_pcd, t)
        avg_rmse += rmse/8
        if rmse < best_rmse:
            best_rmse = rmse
            best_t = t
    
    return best_t, best_rmse


def frustum_hidden_point_removal(pcd, camera_pos):
    frustum_mean = np.mean(np.asarray(pcd.points), axis=0)
    radius = np.linalg.norm(frustum_mean - camera_pos) * 300
    _, pt_map = pcd.hidden_point_removal(camera_pos, radius)
    pcd = pcd.select_by_index(pt_map)
    return pcd



def iterative_icp(obj_pcd, scene_pcd, inital_T):
    obj_pcd = copy.deepcopy(obj_pcd)
    T = inital_T.copy()
    obj_pcd.transform(T)

    for mcd in [4, 2, 1, 0.5, 0.1]:
        icp_result = o3d.pipelines.registration.registration_icp(
            obj_pcd, scene_pcd, max_correspondence_distance=mcd,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
        obj_pcd.transform(icp_result.transformation)
        T = icp_result.transformation @ T
        rmse = icp_result.inlier_rmse
    return T, rmse


