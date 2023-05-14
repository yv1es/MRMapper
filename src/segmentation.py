#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

import mxnet as mx
from gluoncv import model_zoo, data, utils
import matplotlib.pyplot as plt

import struct
import threading
import queue
import open3d as o3d
import numpy as np
import os
import datetime
import cv2
import tf
import os
import time



net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)

FREQ = 5

# queue with (image, camera transform, point cloud)
triplet_queue = queue.Queue()
lock = threading.Lock()
capture = False
cloud_map_msg = None
odom_msg = None
image_msg = None
currently_capturing = False
bridge = CvBridge()
running = True
yolo = None


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
    colors = np.column_stack((b_vals, g_vals, r_vals))

    # Create an Open3D point cloud from the points and colors arrays
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(points)
    pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors / 255.0)

    return pointcloud_o3d


def transform_from_odom(odom):
    # Extract position and orientation information from Odometry message
   
    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation    
    # Convert orientation from quaternion to euler angles
    euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    # Create 4x4 transform matrix
    T = tf.transformations.compose_matrix(translate=[position.x, position.y, position.z], angles=euler)
    return T


def capture_callback(event):
    global lock

    # Attempt to acquire the lock
    if not lock.acquire(blocking=False):
        return
    else:
        try:
            start_capture()
        finally:
            lock.release()


def start_capture():
    global capture, cloud_map_msg, odom_msg, image_msg, lock
    capture = False
    cloud_map_msg= None
    odom_msg = None
    image_msg = None
    
    capture = True
    while cloud_map_msg is None or odom_msg is None or image_msg is None: 
        continue # wait until a triplet is complete

    # make o3d point cloud
    pcd = ros_pointcloud_to_o3d(cloud_map_msg)
    
    # make openCV image
    img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # make transform matrix
    T = transform_from_odom(odom_msg)

    # put in queue
    # triplet_queue.put((img, T, pcd))
    rospy.loginfo("Captured triplet")
    process_triplet(img, T, pcd)




def process_triplet(img, T, pcd):
    start_time = time.time()

    # class_ids, confidences, boxes = yolo.predict(img)
    
    x, img = data.transforms.presets.yolo.load_test(img, short=512)

    # Run object detection
    class_ids, confidences, boxes = net(x)

    
    
    end_time = time.time()
    elapsed_time = end_time - start_time
    rospy.loginfo(f"YOLO Elapsed time: {elapsed_time} seconds")
    
    

    clouds = []
    patches = []

    for i, box in enumerate(boxes):
        class_id = class_ids[i]
        confidence = confidences[i]
        print(yolo.classes[class_id], confidence, box)
        pcd_bb = pcd_from_bb(box, T, pcd)
        clouds.append(pcd_bb)
        patches += detect_planar_patches(pcd_bb)

        x1, y1, x2, y2 = box
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    
    cv2.imshow('Image with bounding box', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # o3d.visualization.draw_geometries([pcd] + patches)
    # if len(patches) > 0:
    #     o3d.visualization.draw_geometries([pcd, patches[0]])
    



def main():
    global yolo
    yolo = Yolo()

    rospy.init_node('segmentation_node', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)

    timer = rospy.Timer(rospy.Duration(FREQ), capture_callback)

    # while running:
    #     try:
    #         img, T, pcd = triplet_queue.get(timeout=1)
    #         process_triplet(img, T, pcd)
    #         triplet_queue.task_done()
    #     except queue.Empty:
    #         pass
    rospy.spin()








def pcd_from_bb(box, extrinsics, pcd):
   
    
    # load intrinsic 
    fx, fy = 618.2962646484375, 617.8786010742188
    cx, cy = 316.1949462890625, 242.33355712890625
    width, height = 640, 480
    intrinsics = np.array([[fx, 0, cx], 
                            [0, fy, cy], 
                            [0, 0, 1 ]])


    x1, y1, x2, y2 = box
    
    object_corners =  np.array([
        [x1, y1],
        [x2, y1],
        [x2, y2],
        [x1, y2]
    ])

    
    object_frustum = frustum_mesh_from_image_coords(object_corners, 5, intrinsics, extrinsics, width, height)
    object_ls = o3d.geometry.LineSet.create_from_triangle_mesh(object_frustum)
    object_ls.paint_uniform_color((0, 0, 1))

    mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_frustum)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh) 

    signed_distance = scene.compute_signed_distance(np.array(pcd.points).astype(np.float32))
    sd = signed_distance.numpy()
    pcd_bb = pcd.select_by_index(np.where(sd <= 0)[0])
    return pcd_bb

    # o3d.visualization.draw_geometries([pcd, object_ls])
    # o3d.visualization.draw_geometries([pcd_corp, object_ls])
    
    # planar_patches = detect_planar_patches(pcd_corp)
    # o3d.visualization.draw_geometries([pcd_corp, object_ls] + planar_patches)
    # o3d.visualization.draw_geometries([pcd, object_ls] + planar_patches)


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













class Yolo():

    def __init__(self) -> None:
        current_path = os.path.dirname(os.path.abspath(__file__))
        yolo_dir = os.path.join(current_path, 'yolo') 
        classes_path = os.path.join(yolo_dir, 'yolov3.txt')
        weights_path = os.path.join(yolo_dir, 'yolov3.weights')
        config_path = os.path.join(yolo_dir, 'yolov3.cfg')

        # read class names from text file
        with open(classes_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]        

        self.net = cv2.dnn.readNet(weights_path, config_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.scale = 0.00392



    def predict(self, img):
        Width = img.shape[1]
        Height = img.shape[0]
        blob = cv2.dnn.blobFromImage(img, self.scale, (416,416), (0,0,0), True, crop=False)

        # set input blob for the network
        self.net.setInput(blob)

        # run inference through the network
        # and gather predictions from output layers
        outs = self.net.forward(self.get_output_layers(self.net))

        # initialization
        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.6
        nms_threshold = 0.4

        # for each detetion from each output layer 
        # get the confidence, class id, bounding box params
        # and ignore weak detections (confidence < 0.5)
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > conf_threshold:
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        return class_ids, confidences, boxes

    def get_output_layers(self, net):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        running = False


