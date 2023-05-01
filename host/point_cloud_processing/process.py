import os
import ast
import sys
import open3d as o3d
import numpy as np
import cv2

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
    # frame_corners = np.array([
    #     [0, 0], 
    #     [0, height-1], 
    #     [width-1, height-1], 
    #     [width-1, 0]])    


    # bild
    object_corners = np.array([
        [104,15],
        [280,15],
        [280,200],
        [110,200],
    ])

    # couch 
    object_corners = np.array([
        [210, 470],
        [537, 109],
        [707, 260],
        [376, 635],
    ])

    # bett
    # object_corners = np.array([
    #     [6,210],
    #     [590,208],
    #     [585,470],
    #     [11, 470],
    # ])

    
    object_frustum = frustum_mesh_from_image_coords(object_corners, 5, intrinsics, extrinsics, width, height)
    object_ls = o3d.geometry.LineSet.create_from_triangle_mesh(object_frustum)
    object_ls.paint_uniform_color((0, 0, 1))

    mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_frustum)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh) 

    signed_distance = scene.compute_signed_distance(np.array(pcd.points).astype(np.float32))
    sd = signed_distance.numpy()
    pcd_corp = pcd.select_by_index(np.where(sd <= 0)[0])


    # o3d.visualization.draw_geometries([pcd, object_ls])
    # o3d.visualization.draw_geometries([pcd_corp, object_ls])
    
    planar_patches = detect_planar_patches(pcd_corp)
    # o3d.visualization.draw_geometries([pcd_corp, object_ls] + planar_patches)
    o3d.visualization.draw_geometries([pcd, object_ls] + planar_patches)



def detect_planar_patches(pcd):

    pcd.estimate_normals()

    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=70,
        coplanarity_deg=70,
        outlier_ratio=0.75,
        min_plane_edge_length=1,
        min_num_points=0,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    print("Detected {} patches".format(len(oboxes)))

    geometries = []
    for obox in oboxes:
        mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
        mesh.paint_uniform_color(obox.color)
        geometries.append(mesh)
        # geometries.append(obox)
    return geometries



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
