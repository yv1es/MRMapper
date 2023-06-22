import os
import ast
import sys
import open3d as o3d
import numpy as np
import cv2
import copy
import matplotlib.pylab as plt

def main():
    # Get the folder path from the command line argument
    if len(sys.argv) < 2:
        print("Usage: python visualize_point_cloud.py folder_path")
        sys.exit(1)
    folder_path = sys.argv[1]
    pcd_path = os.path.join(folder_path, "point_cloud.ply")
    image_path = os.path.join(folder_path, "image.png")
    odom_path = os.path.join(folder_path, "odom.npy")

    # Check if the folder exists and contains the required files
    if not os.path.exists(folder_path):
        print(f"Folder '{folder_path}' does not exist")
        sys.exit(1)
    if not os.path.isfile(image_path):
        print("File 'image.png' not found in the folder")
        sys.exit(1)
    if not os.path.isfile(odom_path):
        print("File 'odom.txt' not found in the folder")
        sys.exit(1)
    if not os.path.isfile(pcd_path):
        print("File 'point_cloud.ply' not found in the folder")
        sys.exit(1)

    pcd = o3d.io.read_point_cloud(os.path.join(folder_path, "point_cloud.ply"))
    

    # load extrinsic
    extrinsics = np.load(odom_path)
    extrinsic = o3d.camera.PinholeCameraParameters()
    extrinsic.extrinsic = extrinsics
    
    # load intrinsic 
    fx, fy = 618.2962646484375, 617.8786010742188
    cx, cy = 316.1949462890625, 242.33355712890625
    width, height = 640, 480
    intrinsics = np.array([[fx, 0, cx], 
                            [0, fy, cy], 
                            [0, 0, 1 ]])

    
    # define camera frustum 
    frame_corners = np.array([
        [0, 0], 
        [0, height-1], 
        [width-1, height-1], 
        [width-1, 0]])    


    # bounding box
    top_left = [0,0]
    bottom_right = [228, 428]

    # top_left = [245, 84]
    # bottom_right = [445, 480]

    # top_left = [211, 7]
    # bottom_right = [422, 422]   

    # top_left = [225, 30]
    # bottom_right = [450, 470]

    object_corners = np.array([
        top_left, 
        [bottom_right[0], top_left[1]],
        bottom_right,
        [top_left[0], bottom_right[1]]
    ])

    

    chair_mesh = o3d.io.read_triangle_mesh("56.obj")

    
    
    
    object_frustum = frustum_mesh_from_image_coords(object_corners, 5, intrinsics, extrinsics, width, height)
    object_ls = o3d.geometry.LineSet.create_from_triangle_mesh(object_frustum)
    object_ls.paint_uniform_color((0, 0, 1))

    mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_frustum)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh) 

    signed_distance = scene.compute_signed_distance(np.array(pcd.points).astype(np.float32))
    sd = signed_distance.numpy()
    point_cloud = pcd.select_by_index(np.where(sd <= 0)[0])

    camera_pos = extrinsics[0:3, 3]

    # o3d.visualization.draw_geometries([pcd, object_ls])
    # o3d.visualization.draw_geometries([pcd, object_ls])

    o3d.visualization.draw_geometries([point_cloud])
    point_cloud = frustum_hidden_point_removal(point_cloud, camera_pos)
    o3d.visualization.draw_geometries([point_cloud])
    




    point_cloud = keep_largest_cluster(point_cloud)







    exit()
    # the objects point cloud 
    chair_cloud = chair_mesh.sample_points_uniformly(number_of_points=1000)

    t, rmse = icp_fit_object(chair_cloud, point_cloud, camera_pos)

    print(rmse)

    # Apply the final tranformation to the chair mesh
    transformed_chair_mesh = chair_mesh.transform(t)

    # Visualize the aligned point cloud and mesh
    o3d.visualization.draw_geometries([transformed_chair_mesh, pcd])
    


def keep_largest_cluster(pcd, k=1):
    #  clustering
    labels = np.array(pcd.cluster_dbscan(eps=0.075, min_points=4))
    label_counts = np.bincount(labels[labels != -1])

    most_common_label = np.argmax(label_counts)
    
    indices = np.where(labels == most_common_label)[0]

    pcd = pcd.select_by_index(indices)

    print(labels)
    print(label_counts)
    print(most_common_label)
    print(indices)

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])
    return pcd



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


def move_mesh_to_pos_transform(mesh, pos):
    translation = pos - mesh.get_center()
    t = np.eye(4)
    t[0:3, 3] = translation
    return t


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



if __name__ == "__main__":
    main()
