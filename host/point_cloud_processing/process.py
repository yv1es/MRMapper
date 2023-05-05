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

    
    # define thrustum 
    corner_points = np.array([
        [0, 0], 
        [0, height-1], 
        [width-1, height-1], 
        [width-1, 0]])    
    frustum = image_points_to_direction(corner_points, intrinsics, extrinsics, width, height) 
    frustum *= 5

    objt_points = np.array([
        [108,13],
        [281,57],
        [280,204],
        [112,190],
        [112,212]
    ])

    objt = image_points_to_direction(objt_points, intrinsics, extrinsics, width, height) 
    objt *= 5

    geoms = [pcd]

    draw_vecs(frustum, geoms, [0, 0, 1], extrinsics)
    draw_vecs(objt, geoms, [0,1,0], extrinsics)    

    o3d.visualization.draw_geometries(geoms)



def draw_vecs(vectors, geoms, color, extrinsics):
    for i in range(vectors.shape[0]):
            start = np.zeros(3)
            end = vectors[i] 
            line = o3d.geometry.LineSet()
            line.points = o3d.utility.Vector3dVector(np.array([start, end]))
            line.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
            line.paint_uniform_color(color)
            line.transform(extrinsics)
            geoms.append(line)



def image_points_to_direction(points, intrinsics, extrinsics, width, height):
    """Takes an array of 2D point on the image plane and returns an array of 3D vectors 
    which points from the camera into the direction of the point. 

    Args:
        points (n x 2 numpy array): 2D points  
        intrinsics (3x3 numpy array): intrinsics matrix
        extrinsics (4x4 numy array): extrinsics matrix
        width (int): image plane width
        height (int): image plane height
    """
    # camera parameters
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
