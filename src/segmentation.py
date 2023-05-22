#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import socket as s 

import mxnet as mx
from gluoncv import model_zoo, data
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



FRAME_WIDTH = 640
FRAME_HEIGHT = 480

HOST = s.gethostname() 
PORT_PLANES = 5003

FREQ_PLANE_EXTRACTION = 1
FREQ_SEND_PLANES = 1

net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)


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
    # if capture and cloud_map_msg is None:
        # cloud_map_msg = data
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
    # cloud_map_msg= None
    odom_msg = None
    image_msg = None
    
    capture = True
    while cloud_map_msg is None or odom_msg is None or image_msg is None: 
        continue # wait until a triplet is complete

    # convert from ROS to open3d point cloud 
    pcd = ros_pointcloud_to_o3d(cloud_map_msg)
    # pcd = orh.rospc_to_o3dpc(cloud_map_msg) 

    # convert from ROS to opencv image
    img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    # img = imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # make the camera transform matrix 
    T = transform_from_odom(odom_msg)

    rospy.loginfo("Captured triplet")

    # segmentation and plane fitting
    process_triplet(img, T, pcd)





p = np.empty((0, 4, 3))


def process_triplet(img, T, pcd):
    global p
    print(img.shape)
    assert (img.shape == (FRAME_HEIGHT, FRAME_WIDTH, 3))
    start_time = time.time()

    print("Transform image for yolo")
    # prepare image for yolo
    x, tmp_img = data.transforms.presets.yolo.transform_test(mx.nd.array(img), short=512)

    # compute scale factors of the transform above
    scale = (np.array(img.shape) / np.array(tmp_img.shape))[:2]

    print("Predicting with yolo")
    # predict bounding boxes with yolo
    class_ids, confidences, boxes = net(x)

    print("Converting prediction")
    # convert from MXnet arrays to numpy arrays with reasonable shape
    class_ids = class_ids.asnumpy().reshape(100, 1).astype(np.int32)
    confidences = confidences.asnumpy().reshape(100, 1)
    boxes = boxes.asnumpy().reshape(100, 4)

    # undo the yolo input transformation
    boxes[:, 0] *= scale[0]
    boxes[:, 2] *= scale[0]
    boxes[:, 1] *= scale[1]
    boxes[:, 3] *= scale[1]

    # filter for predictions above a treshold 
    idx = np.where(confidences < 0.1)[0][0]
    class_ids = class_ids[:idx]
    confidences = confidences[:idx]
    boxes = boxes[:idx]

    

    print("Entering loop over boxes")
    for i in range(len(class_ids)):
        class_id = class_ids[i]
        confidence = confidences[i]
        box = boxes[i]

        if not (int(class_id) in flat):
            # continue
            pass
        print("Extract box")

        # print info about this bounding box
        print(class_id, confidence, box, classes[int(class_id)])

        x1, y1, x2, y2 = box

        edge_threshold = 5
        if x1 < edge_threshold or y1 < edge_threshold: 
            print("box to close to border")
            continue
            # pass

        if img.shape[1] - x2 < edge_threshold or img.shape[0] - y2 < edge_threshold:
            print("box to close to border")
            continue
            # pass

        print("Cropping frustum from point cloud")
        # project its frustum into 3D and make a point cloud with all points inside
        pcd_bb = pcd_from_bb(box, T, pcd)

        print("Checking point cloud for enough inliers")
        # check that the point cloud is not to empty 
        if  len(pcd_bb.points) < 20:
            continue 
            # pass
        
        print("Detecting planar patches")
        # oboxes = detect_planar_patches(pcd_bb)
        _, plane_bb = segment_plane(pcd_bb)
        oboxes = [plane_bb]
        print("Detection of patches done")

        print("Saving planar patches")
        for obox in oboxes: 
            corners = obox_to_corners(obox).reshape((1, 4, 3))
            print("Stacking corner points")

            squared_diff = (p - corners) ** 2
            sum_squared_diff = np.sum(np.sum(squared_diff, axis=1), axis=1)

            ALPHA = 0.1
            print(sum_squared_diff)

            if sum_squared_diff.shape[0] > 0 and np.min(sum_squared_diff) < 7.5: # duplicate plane check
                
                print("Updating plane")
                idx = np.argmin(sum_squared_diff)
                old = p[idx, : , :]
                p[idx, : , :] = old * (1-ALPHA) + corners * ALPHA 
                
                
                continue
            
            print("Adding plane")
            # add new corner
            p = np.vstack([p, corners])

            

    print("Sending planes")
    send_planes(p)

    # timing of the plane detection pipeline
    end_time = time.time()
    elapsed_time = end_time - start_time
    rospy.loginfo(f"YOLO Elapsed time: {elapsed_time} seconds")
    
    
    

def draw_boxes(image, class_ids, confidences, boxes, classes):
    # Define some colors
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # Loop over all detections and draw the bounding boxes
    for class_id, confidence, box in zip(class_ids, confidences, boxes):
        x1, y1, x2, y2 = box.astype(np.int32)

        # Get the class label and color
        class_label = classes[int(class_id)]
        color = colors[int(class_id)]

        # Draw the bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        # Draw the label and confidence
        label = "{}: {}".format(class_label, confidence)
        cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return image
    


def obox_to_corners(obb):
    print("Computing corner points")
    plane = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obb, scale=[1, 1, 1e-30])
    print("Creted mesh from bounding box")
    points = np.asarray(plane.vertices)
    points = np.unique(points, axis=0)
    return points







def segment_plane(pcd):

    
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.002,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1, 0, 0])
    

    # remove outliers
    cl, ind = inlier_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    # cl, ind = inlier_cloud.remove_radius_outlier(nb_points=4, radius=0.1)
    inlier_cloud = cl.select_by_index(ind)

    # clustering
    # labels = np.array(inlier_cloud.cluster_dbscan(eps=0.25, min_points=10))
    # label_counts = np.bincount(labels[labels != -1])
    # most_common_label = np.argmax(label_counts)
    
    # indices = np.where(labels == most_common_label)[0]

    # inlier_cloud = inlier_cloud.select_by_index(indices)

    # print(labels)
    # print(label_counts)
    # print(most_common_label)
    # print(indices)

    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # inlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([inlier_cloud])


    # hull, _ = inlier_cloud.compute_convex_hull()

    # hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    # hull_ls.paint_uniform_color((1, 0, 0))
    # find bounding box     
    obb = o3d.geometry.OrientedBoundingBox.create_from_points(inlier_cloud.points)
    # obb = o3d.geometry.OrientedBoundingBox.get_minimal_oriented_bounding_box(inlier_cloud)

    # plane = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obb, scale=[1, 1, 0.0001])
    # plane.paint_uniform_color((0, 1, 0))

    # plane = plane_mesh_from_obb(obb)

    # ls =  o3d.geometry.LineSet.create_from_oriented_bounding_box(obb)

    # ls.paint_uniform_color((0, 1, 0))
    
    return inlier_cloud, obb



def detect_planar_patches(pcd):
    
    print("Estimating normals")
    pcd.estimate_normals()

    print("Removing outliers")
    # remove outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    # cl, ind = inlier_cloud.remove_radius_outlier(nb_points=4, radius=0.1)
    pcd = cl.select_by_index(ind)

    print("Invoke open3d detect planar patches")
    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=70,
        coplanarity_deg=70,
        outlier_ratio=0.5,
        min_plane_edge_length=0.5,
        min_num_points=20,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    print("Detected {} patches".format(len(oboxes)))

    # geometries = []
    # for obox in oboxes:
    #     geometries.append(obox)
        # geometries.append(obox)
    return oboxes






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



planes_temp = np.array([ [ [0., 0., 1.], [0., 1., 0.], [0., 1., 1.], [0., 0., 0.],],
                    [[1., 0., 0.],  [1., 1., 0.], [1., 0., 1.], [1., 1., 1.]] ]).astype(np.float32)
    

def send_planes(planes):
    print("Corners:")
    print(planes)
    planes = planes.astype(np.float32)
    data = planes.tobytes()
    sender_planes.send(data)
    rospy.loginfo("Sent planes")




def main():
    global sender_planes
    sender_planes = UnitySender(HOST, PORT_PLANES, 'Plane Sender')

    sender_planes.start()

    rospy.loginfo("Connected to unity")
    
    rospy.init_node('segmentation_node', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)

    rospy.Timer(rospy.Duration(FREQ_PLANE_EXTRACTION), capture_callback)
    # rospy.Timer(rospy.Duration(FREQ_SEND_PLANES), send_planes)

    rospy.spin()







class UnitySender:
    def __init__(self, ip, port, tag) -> None:        
        self.ip = ip 
        self.port = port 
        self.tag = tag
        self.sender_queue = queue.Queue()
        self.running = True
        self.conn = None
        

    def start(self):
        self.socket = s.socket(s.AF_INET, s.SOCK_STREAM)
        self.socket.bind((self.ip, self.port)) 
        self.socket.listen()
        self.log("Listening incoming unity connection")
        self.conn, _ = self.socket.accept()
        self.log("Connected to unity")

        self.sender = threading.Thread(target=self.senderThread)
        self.sender.daemon = True
        self.sender.start()
        self.log("Sender thread running")


    def log(self, message):
        rospy.loginfo("[{}] {}".format(self.tag, message))


    def senderThread(self):
        while self.running:
                try:
                    data = self.sender_queue.get(timeout=1)
                    self.conn.sendall(data)
                    self.sender_queue.task_done()
                except queue.Empty:
                    pass


    def send(self, data):
        header = struct.pack('!I', len(data))
        message = header + data
        self.sender_queue.put(message)


    def stop(self):
        self.running = False
        self.conn.close()
        self.socket.close()




















classes = [ 'person',
    'bicycle',
    'car',
    'motorcycle',
    'airplane',
    'bus',
    'train',
    'truck',
    'boat',
    'traffic light',
    'fire hydrant',
    'stop sign',
    'parking meter',
    'bench',
    'bird',
    'cat',
    'dog',
    'horse',
    'sheep',
    'cow',
    'elephant',
    'bear',
    'zebra',
    'giraffe',
    'backpack',
    'umbrella',
    'handbag',
    'tie',
    'suitcase',
    'frisbee',
    'skis',
    'snowboard',
    'sports ball',
    'kite',
    'baseball bat',
    'baseball glove',
    'skateboard',
    'surfboard',
    'tennis racket',
    'bottle',
    'wine glass',
    'cup',
    'fork',
    'knife',
    'spoon',
    'bowl',
    'banana',
    'apple',
    'sandwich',
    'orange',
    'broccoli',
    'carrot',
    'hot dog',
    'pizza',
    'donut',
    'cake',
    'chair',
    'couch',
    'potted plant',
    'bed',
    'dining table',
    'toilet',
    'tv',
    'laptop',
    'mouse',
    'remote',
    'keyboard',
    'cell phone',
    'microwave',
    'oven',
    'toaster',
    'sink',
    'refrigerator',
    'book',
    'clock',
    'vase',
    'scissors',
    'teddy bear',
    'hair drier',
    'toothbrush']

flat = {59, 60, 66, 67, 62}




import sys
import numpy as np
from sensor_msgs.msg import Image

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        running = False


