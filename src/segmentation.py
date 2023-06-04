#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import mxnet as mx
from gluoncv import model_zoo, data
import threading
import open3d as o3d
import numpy as np
import cv2
import tf
import time

from unity_sender import UnitySender 
from point_cloud_processing import ros_pointcloud_to_o3d, pcd_from_bbox, obox_to_corners
from constants import *


net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)
planes = np.empty((0, 4, 3))
plane_labels = np.empty((0,1))
lock = threading.Lock()
capture = False
bridge = CvBridge()

# store simultaniously captured messages
cloud_map_msg = None
odom_msg = None
image_msg = None


def cloud_map_callback(data):
    global cloud_map_msg
    cloud_map_msg = data

def odom_callback(data):
    global odom_msg
    if capture and odom_msg is None:
        odom_msg = data

def image_callback(data):
    global image_msg
    if capture and image_msg is None:
        image_msg = data

def capture_callback(event):
    global lock

    # Launch a capture, locking prevents multiple captures running in parallel
    if not lock.acquire(blocking=False):
        return
    else:
        try:
            start_capture()
        finally:
            lock.release()


def transform_matrix_from_odom(odom_msg):
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation    
    euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    T = tf.transformations.compose_matrix(translate=[position.x, position.y, position.z], angles=euler)
    return T


def start_capture():
    global capture, cloud_map_msg, odom_msg, image_msg, lock
    capture = False
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
    T = transform_matrix_from_odom(odom_msg)

    rospy.loginfo("Captured triplet")
    process_capture(img, T, pcd)


def segment_plane(pcd):
    _, inliers = pcd.segment_plane(distance_threshold=0.002, ransac_n=3, num_iterations=1000)

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1, 0, 0])
    
    # remove outliers
    cl, ind = inlier_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    # cl, ind = inlier_cloud.remove_radius_outlier(nb_points=4, radius=0.1)
    inlier_cloud = cl.select_by_index(ind)

    obox = o3d.geometry.OrientedBoundingBox.create_from_points(inlier_cloud.points)
    fit_rate = len(inlier_cloud.points) / len (pcd.points)
    rospy.loginfo("Fit rate {}".format(fit_rate))
    return inlier_cloud, obox, fit_rate



def detect_planar_patches(pcd):    
    pcd.estimate_normals()

    # remove outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    # cl, ind = inlier_cloud.remove_radius_outlier(nb_points=4, radius=0.1)
    pcd = cl.select_by_index(ind)

    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=70,
        coplanarity_deg=70,
        outlier_ratio=0.5,
        min_plane_edge_length=0.5,
        min_num_points=20,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    return oboxes



def yolo_predict(img):
    # prepare image for yolo
    x, tmp_img = data.transforms.presets.yolo.transform_test(mx.nd.array(img), short=512)

    # compute scale factors of the transform above
    scale = (np.array(img.shape) / np.array(tmp_img.shape))[:2]

    rospy.loginfo("Predicting with yolo")
    # predict bounding boxes with yolo
    class_ids, confidences, boxes = net(x)

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
    idx = np.where(confidences < YOLO_CONFIDENCE_TRESHOLD)[0][0]
    class_ids = class_ids[:idx]
    confidences = confidences[:idx]
    boxes = boxes[:idx]
    return class_ids, confidences, boxes



def process_capture(img, T, pcd):
    global planes, plane_labels
    
    start_time = time.time()
    
    # yolo object detection
    class_ids, confidences, boxes = yolo_predict(img)
    rospy.loginfo("Yolo found {} objects:".format(len(class_ids)))
    for i in range(len(class_ids)):
        class_id = class_ids[i]
        confidence = confidences[i]
        bbox = boxes[i]
        print("\t {}, {}, {}, {}".format(class_id, confidence, bbox, CLASSES[int(class_id)]))

    for i in range(len(class_ids)):
        class_id = class_ids[i]
        confidence = confidences[i]
        bbox = boxes[i]

        # only proceed with flat objects
        if not (int(class_id) in FLAT):
            # continue
            pass

        # only proceed when the bounding box has MIN_BOUNDING_BOX_MARGIN pixels distance from the frame borders
        x1, y1, x2, y2 = bbox
        if x1 < MIN_BOUNDING_BOX_MARGIN or y1 < MIN_BOUNDING_BOX_MARGIN: 
            continue

        if img.shape[1] - x2 < MIN_BOUNDING_BOX_MARGIN or img.shape[0] - y2 < MIN_BOUNDING_BOX_MARGIN:
            continue

        # project its frustum into 3D and make a point cloud with all points inside
        pcd_bbox = pcd_from_bbox(bbox, T, pcd)

        # only proceed when the cut out point cloud has enough points
        if  len(pcd_bbox.points) < 20:
            continue 
        
        # oboxes = detect_planar_patches(pcd_bbox)
        _, plane_bb, fit_rate = segment_plane(pcd_bbox)
        oboxes = [plane_bb]

        
        
        rospy.loginfo("Found {} planes".format(len(oboxes)))

        for obox in oboxes:
            # compute the 4 corner points from planes obox 
            corners = obox_to_corners(obox).reshape((1, 4, 3))

            # computing if a similar plane was already registered
            squared_diff = (planes - corners) ** 2
            sum_squared_diff = np.sum(np.sum(squared_diff, axis=1), axis=1)

            
            if sum_squared_diff.shape[0] > 0 and np.min(sum_squared_diff) < MIN_PLANE_DISTANCE: # duplicate plane check
                # update the most similar plane
                idx = np.argmin(sum_squared_diff)
                old = planes[idx, : , :]
                planes[idx, : , :] = old * (1-PLANE_UPDATE_WEIGHT) + corners * PLANE_UPDATE_WEIGHT 

            # else add a new plane if the fit rate is good enough
            elif fit_rate >= MIN_FIT_RATE:
                planes = np.vstack([planes, corners])
                plane_labels = np.vstack([plane_labels, np.array(class_id)])

    # send planes to unity 
    send_planes(planes, plane_labels)

    # timing of the plane extraction pipeline
    end_time = time.time()
    elapsed_time = end_time - start_time
    rospy.loginfo(f"Plane extraction elapsed time: {elapsed_time} seconds")
    
    


def send_planes(planes, plane_labels):
    planes = planes.astype(np.float32)
    plane_labels = plane_labels.astype(np.int32)
    data = planes.tobytes() + plane_labels.tobytes()
    plane_sender.send(data)
    plane_sender.log("Sent {} planes to unity".format(planes.shape[0]))



def main():
    global plane_sender
    plane_sender = UnitySender(HOST, PORT_PLANES, 'Plane Sender')
    plane_sender.start()

    rospy.init_node('segmentation_node', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)

    rospy.Timer(rospy.Duration(FREQ_PLANE_EXTRACTION), capture_callback)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        plane_sender.stop()


