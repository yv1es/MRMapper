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

    # Load the point cloud and visualize it
    pcd = o3d.io.read_point_cloud(os.path.join(folder_path, "point_cloud.ply"))
    # o3d.visualization.draw_geometries([pcd])

    extrinsics = np.load(odom_path)
    # Load the odometry data
    # with open(odom_path, "r") as f:
    # #     odom = my_list = ast.literal_eval(f.readline())
    # #     translation = np.array(odom[:3])
    # #     ros_quaternion = np.array(odom[3:])  
    #     extrinsics = string_to_array(f.readlines())
    
    print(extrinsics)
    # x = ros_quaternion[0] 
    # y = ros_quaternion[1]
    # z = ros_quaternion[2]
    # w = ros_quaternion[3]
    
    # quaternion = ros_quaternion.copy()
    # quaternion[0] = w
    # quaternion[1] = x
    # quaternion[2] = z
    # quaternion[3] = y

    # rotation = o3d.geometry.get_rotation_matrix_from_quaternion(quaternion)


    # rotation =  rotation

    # R = np.eye(4)
    # R[:3, :3] = rotation

    # T = np.eye(4)
    # T[:3, 3] = translation

    # # camera transform 
    # extrinsics = T @ R
    extrinsic = o3d.camera.PinholeCameraParameters()
    extrinsic.extrinsic = extrinsics
    
    # Create the PinholeCameraIntrinsic object with the new parameters
    # intrinsic = o3d.camera.PinholeCameraIntrinsic(width=640, height=480, fx=618.2962646484375,
    #                                             fy=617.8786010742188, cx=316.1949462890625, cy=242.33355712890625)


    fx, fy = 618.2962646484375, 617.8786010742188
    cx, cy = 316.1949462890625, 242.33355712890625
    width, height = 640, 480


    intrinsics = np.array([[fx, 0, cx], 
                            [0, fy, cy], 
                            [0, 0, 1 ]])

    # # Compute image frame corners in pixel coordinates
    # corners_px = np.array([ [0, 0], 
    #                         [0, height-1], 
    #                         [width-1, height-1], 
    #                         [width-1, 0]]).astype(float)

    # corners_px[:,0] = (corners_px[:,0] - cx) / fx
    # corners_px[:,1] = (corners_px[:,1] - cy) / fy 

    # vectors = np.hstack([-np.ones((corners_px.shape[0], 1)), corners_px ])
    # norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    # vectors = vectors / norms

    # # vectors *= 5
    # print(vectors )    

    

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

    # objt_points = objt_points_c
    # objt_points[:,0] = objt_points_c[:,1]
    # objt_points[:,1] = objt_points_c[:,0]

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


# def string_to_array(string):
#     string = "[[ 0.90208868 -0.29627453  0.31377923 -0.1323075 ] [ 0.31554491  0.94884391 -0.0112538  -0.00669084] [-0.2943933   0.10916337  0.94942927  0.05401767] [ 0.          0.          0.          1.        ]]"

#     # Remove the square brackets at the beginning and end of the string
#     string = string.strip('[]')

#     # Replace the inner square brackets with commas to make it a valid numpy array
#     string = string.replace('[', '').replace(']', '').replace('\n', '')

#     # Split the string into a list of strings, with each string representing a row in the array
#     rows = string.split(' ')

#     # Convert the list of strings into a 2D numpy array
#     array = np.array([np.fromstring(row, sep=' ') for row in rows])
#     return array


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

    # cube.lines = o3d.utility.Vector2iVector(edges)

    # K = intrinsic.intrinsic_matrix
    # K = np.hstack([K, np.zeros((3, 1))])
    # E = extrinsics
    # P = K @ E

    # # # projection 
    # # P_inv = np.linalg.pinv(P)

    # # Create frustum mesh
    # fov_h = np.radians(45)
    # fov_v = np.radians(30)
    # near = 0.1
    # far = 10
    # mesh = o3d.geometry.TriangleMesh.create_cone(radius=near*np.tan(fov_h/2), height=near*np.tan(fov_v/2), resolution=4)
    # mesh.translate([0, 0, -near])

    # # Transform mesh using camera intrinsic and extrinsic parameters
    # mesh.transform(np.linalg.inv(K).dot(extrinsics))

    # width = 640
    # height = 480
    # scale = 0.005
    # camera_plane_v = np.array([[0,0,0], [0,0,1], [0,1,0], [0,1,1]]).astype(float)
    # camera_plane_v -= 0.5 # center
    # camera_plane_v[:,1] *= width * scale
    # camera_plane_v[:,2] *= height * scale
    # camera_plane_e = [[0, 1], [1, 3], [3, 2], [2, 0]]

    # camera_plane = o3d.geometry.LineSet()
    # camera_plane.points = o3d.utility.Vector3dVector(camera_plane_v)
    # camera_plane.lines = o3d.utility.Vector2iVector(camera_plane_e)
    # camera_plane.paint_uniform_color([0, 1, 0])
    # camera_plane.transform(extrinsics)

    # o3d.visualization.draw_geometries([pcd, camera_plane])








    # # Define unit cube vertices
    # vertices = np.array([[0, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 1], [1, 0, 0], [1, 0, 1], [1, 1, 0], [1, 1, 1]])
    # vertices = vertices - 0.5
    # vertices[:,0] *= -5
    # vertices = vertices * 0.3

    # # Define unit cube edges
    # edges = np.array([[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4], [0, 4], [1, 5], [2, 6], [3, 7]])

    # # Create Open3D geometry
    # cube = o3d.geometry.LineSet()
    # cube.points = o3d.utility.Vector3dVector(vertices)
    # cube.lines = o3d.utility.Vector2iVector(edges)
    # cube.paint_uniform_color([0, 1, 0])
 
    # cube.transform(extrinsics)

    # # Visualize unit cube
    # # o3d.visualization.draw_geometries([pcd, cube])


    # # Define camera intrinsics
    
    # K = intrinsic.intrinsic_matrix
    # K = np.hstack([K, np.zeros((3, 1))])

    # E = extrinsics

    # # projection 
    # P = K @ E
    # P_inv = np.linalg.pinv(P)

    # # Define image plane coordinates
    # u = 0
    # v = 0
    # q = np.array([u, v, 1])

    # # Compute 3D point on image plane
    # # Q_hom = P_inv @ q
    # # Q = Q_hom[:3] / Q_hom[3]

    # Q_hom0 = P_inv @ np.array([0, 0, 1])
    # Q_hom1 = P_inv @ np.array([640, 0, 1])
    # Q_hom2 = P_inv @ np.array([640, 480, 1])
    # Q_hom3 = P_inv @ np.array([0, 480, 1])

    # Q0 = Q_hom0[:3] / Q_hom0[3]
    # Q1 = Q_hom1[:3] / Q_hom1[3]
    # Q2 = Q_hom2[:3] / Q_hom2[3]
    # Q3 = Q_hom3[:3] / Q_hom3[3]

    # print(Q0, Q1, Q2, Q3)

    # e = np.array([[0, 1], [1,2], [2, 3], [3, 0]])

    # v = [Q0, Q1, Q2, Q3]
    
    # plane = o3d.geometry.LineSet()
    # plane.points = o3d.utility.Vector3dVector(v)
    # plane.lines = o3d.utility.Vector2iVector(e)
    # plane.paint_uniform_color([1, 0, 0])

    # o3d.visualization.draw_geometries([pcd, plane, cube])
    




if __name__ == "__main__":
    main()
