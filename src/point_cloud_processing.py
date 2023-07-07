# !/usr/bin/env python3
import open3d as o3d
import numpy as np
import copy 

from constants import *
from utils import *
from detections import Plane, ICPObject

""" 
This file contains function operating on (Open3d) point clouds that are used mostly in the semantic inference stage. 
"""


def pcd_from_bbox(box, extrinsics, pcd):
    """
    Projects a 2D bouding box on the camera plane into a 3D frustum and creates 
    a point cloud from all points within that frustum. 

    Args:
        box (tuple): The bounding box coordinates (x1, y1, x2, y2)  (left-top, right-bottom points).
        extrinsics (ndarray): The extrinsic matrix (encoding the camera postition).
        pcd (open3d.geometry.PointCloud): The global point cloud.

    Returns:
        open3d.geometry.PointCloud: The point cloud within the frustum spun by the bounding box.
    """
    intrinsics = np.array(CAMERA_K).reshape((3, 3))

    x1, y1, x2, y2 = box
    
    object_corners =  np.array([
        [x1, y1],
        [x2, y1],
        [x2, y2],
        [x1, y2]
    ])

    # compute mesh of frustum goint through the object corners
    object_frustum = frustum_mesh_from_image_coords(object_corners, 10, intrinsics, extrinsics)

    # add mesh for SDF computation 
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_frustum)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh) 

    # compute the SDF of the frustum to make inside outside check 
    signed_distance = scene.compute_signed_distance(np.array(pcd.points).astype(np.float32))
    sd = signed_distance.numpy()
    pcd_bbox = pcd.select_by_index(np.where(sd <= 0)[0])
    return pcd_bbox




def frustum_from_corners(origin, corners, extrinsics):
    """
    Generates a frustum mesh from the given 4 corners.

    Args:
        origin (ndarray): The origin of the frustum relative to the camera (hence normally zero).
        corners (ndarray): The corners of the frustum relative to the camera.
        extrinsics (ndarray): The extrinsic matrix (camear pose).

    Returns:
        open3d.geometry.TriangleMesh: The frustum mesh.
    """
    mesh = o3d.geometry.TriangleMesh()

    faces = np.array([[0, 2, 1], [0, 3, 2], [0, 4, 3], [0, 1, 4], [1, 2, 3], [3, 4, 1]])

    mesh.vertices = o3d.utility.Vector3dVector(np.vstack((origin, corners)))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(faces))
    mesh.transform(extrinsics)
    
    return mesh


# takes 2D points from the image and gives a 3D mesh of the frustum projected in 3d 
def frustum_mesh_from_image_coords(points, frustum_depth, intrinsics, extrinsics):
    """
    Generates a 3D mesh of the frustum projected into 3D from the given 2D image coordinates.

    Args:
        points (ndarray): The 2D points from the image.
        frustum_depth (float): The depth of the frustum.
        intrinsics (ndarray): The intrinsic matrix.
        extrinsics (ndarray): The extrinsic matrix.
        width (int): The width of the image.
        height (int): The height of the image.

    Returns:
        open3d.geometry.TriangleMesh: The frustum mesh.
    """
    vecs = image_points_to_direction(points, intrinsics) 
    vecs *= frustum_depth
    mesh = frustum_from_corners(np.zeros(3), vecs, extrinsics)
    return mesh


def image_points_to_direction(points, intrinsics):
    """
    Computes the direction unit vectors pointing from the camera to the 2D points on the camera plane.

    Args:
        points (ndarray): The image points.
        intrinsics (ndarray): The intrinsic matrix.
        extrinsics (ndarray): The extrinsic matrix.
        width (int): The width of the image.
        height (int): The height of the image.

    Returns:
        ndarray: The direction unit vectors.
    """
    fx, fy = intrinsics[0,0], intrinsics[1,1]
    cx, cy = intrinsics[0,2], intrinsics[1,2]
    
    p = points.astype(float).copy()
    
    p[:,0] = (p[:,0] - cx) / fx
    p[:,1] = (p[:,1] - cy) / fy 

    vectors = np.hstack([-np.ones((p.shape[0], 1)), p])
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    vectors = -vectors / norms

    return vectors



def icp_fit_object(obj_pcd, frustum_pcd, camera_pos, class_id):
    """
    Performs ICP registration to fit the object point cloud to the frustum point cloud.
    Tries out multiple inital transforms and chooses the best based on RMSE. 

    Args:
        obj_pcd (open3d.geometry.PointCloud): The object point cloud.
        frustum_pcd (open3d.geometry.PointCloud): The frustum point cloud.
        camera_pos (ndarray): The camera position.

    Returns:
        ICPObject: The newly fitted ICPObject
    """
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
    
    new_icp_object = ICPObject(best_t, class_id, rmse)
    return new_icp_object


def frustum_hidden_point_removal(pcd, camera_pos):
    """
    Removes hidden points from the point cloud based on the camera position.

    Args:
        pcd (open3d.geometry.PointCloud): The point cloud.
        camera_pos (ndarray): The camera position.

    Returns:
        open3d.geometry.PointCloud: The filtered point cloud.
    """
    frustum_mean = np.mean(np.asarray(pcd.points), axis=0)
    radius = np.linalg.norm(frustum_mean - camera_pos) * 300
    _, pt_map = pcd.hidden_point_removal(camera_pos, radius)
    pcd = pcd.select_by_index(pt_map)
    return pcd



def iterative_icp(obj_pcd, scene_pcd, inital_T):
    """
    Performs iterative ICP registration between the object point cloud and the scene point cloud.

    Args:
        obj_pcd (open3d.geometry.PointCloud): The object point cloud.
        scene_pcd (open3d.geometry.PointCloud): The scene point cloud.
        inital_T (ndarray): The initial transformation matrix.

    Returns:
        tuple: A tuple containing the transformation matrix and the RMSE value.
    """
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



def fit_plane(pcd, class_id):
    """
    Fits a plane to the point cloud and returns the inlier points, oriented bounding box, and fit rate.
    Returns None when the cloud is too sparse

    Args:
        pcd (open3d.geometry.PointCloud): The point cloud.

    Returns:
        Plane: A new plane object.
    """
    _, inliers = pcd.segment_plane(distance_threshold=0.002, ransac_n=3, num_iterations=100000)
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud = keep_largest_cluster(inlier_cloud)
    if (len(inlier_cloud.points) == 0):
        return None
    obox = o3d.geometry.OrientedBoundingBox.create_from_points(inlier_cloud.points)
    fit_rate = len(inlier_cloud.points) / len (pcd.points)
    corners = obox_to_corners(obox).reshape((4, 3))
    new_plane = Plane(corners, class_id, fit_rate)
    return new_plane




def keep_largest_cluster(pcd):
    """
    Keeps the largest cluster from the point cloud using DBSCAN clustering.
    Retruns an empyt point cloud when no cluster was found. 

    Args:
        pcd (open3d.geometry.PointCloud): The point cloud.

    Returns:
        open3d.geometry.PointCloud: The largest cluster point cloud.
    """
    labels = np.array(pcd.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_POINTS))
    label_counts = np.bincount(labels[labels != -1])
    if label_counts.shape[0] == 0:
        return o3d.geometry.PointCloud()
    most_common_label = np.argmax(label_counts)
    indices = np.where(labels == most_common_label)[0]
    pcd = pcd.select_by_index(indices)
    max_label = labels.max()
    return pcd
