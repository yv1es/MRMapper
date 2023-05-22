#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import socket as s 

from unity_sender import UnitySender 

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

from point_cloud_processing import ros_pointcloud_to_o3d, pcd_from_bb, obox_to_corners

import constants






HOST = s.gethostname() 
PORT_PLANES = 5003

FREQ_PLANE_EXTRACTION = 1
FREQ_SEND_PLANES = 1

net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)

p = np.empty((0, 4, 3))


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

    # convert from ROS to opencv image
    img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # make the camera transform matrix 
    T = transform_from_odom(odom_msg)

    rospy.loginfo("Captured triplet")

    # segmentation and plane fitting
    process_triplet(img, T, pcd)






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
    
    
    

# def draw_boxes(image, class_ids, confidences, boxes, classes):
#     # Define some colors
#     colors = np.random.uniform(0, 255, size=(len(classes), 3))

#     # Loop over all detections and draw the bounding boxes
#     for class_id, confidence, box in zip(class_ids, confidences, boxes):
#         x1, y1, x2, y2 = box.astype(np.int32)

#         # Get the class label and color
#         class_label = classes[int(class_id)]
#         color = colors[int(class_id)]

#         # Draw the bounding box
#         cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

#         # Draw the label and confidence
#         label = "{}: {}".format(class_label, confidence)
#         cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

#     return image
    









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










def send_planes(planes):
    planes = planes.astype(np.float32)
    data = planes.tobytes()
    sender_planes.send(data)
    sender_planes.log("Sent {} planes to unity".format(planes.shape[0]))



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




flat = {59, 60, 66, 67, 62}

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








if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        running = False


