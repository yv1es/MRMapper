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
from point_cloud_processing import *
from constants import *


net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)
planes = np.empty((0, 4, 3))
plane_labels = np.empty((0,1))
lock = threading.Lock()
capture = False
bridge = CvBridge()
chair_mesh = load_chair_mesh()

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

    _, inliers = pcd.segment_plane(distance_threshold=0.002, ransac_n=3, num_iterations=100000)

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


    obox = o3d.geometry.OrientedBoundingBox.create_from_points(inlier_cloud.points)
    
    fit_rate = len(inlier_cloud.points) / len (pcd.points)
    fit_rate = min(FIT_RATE_NORMALIZATION, fit_rate) / FIT_RATE_NORMALIZATION 

    rospy.loginfo("Fit rate {}".format(fit_rate))
    return inlier_cloud, obox, fit_rate





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



def process_capture(img, camera_T, pcd):
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

    # loop over detected objects
    for i in range(len(class_ids)):
        class_id = class_ids[i]
        confidence = confidences[i]
        bbox = boxes[i]


        # only proceed when the bounding box has MIN_BOUNDING_BOX_MARGIN pixels distance from the frame borders
        x1, y1, x2, y2 = bbox
        if x1 < MIN_BOUNDING_BOX_MARGIN or y1 < MIN_BOUNDING_BOX_MARGIN: 
            continue

        if img.shape[1] - x2 < MIN_BOUNDING_BOX_MARGIN or img.shape[0] - y2 < MIN_BOUNDING_BOX_MARGIN:
            continue

        # project its frustum into 3D and make a point cloud with all points inside
        pcd_bbox = pcd_from_bbox(bbox, camera_T, pcd)

        # remove points that should be hidden from the camera
        pcd_bbox = frustum_hidden_point_removal(pcd_bbox, camera_pos)

        # only proceed when the cut out point cloud has enough points
        if  len(pcd_bbox.points) < 20:
            continue 
       
        
        if int(class_id) == CHAIR: 
            print("CHAIR")
            camera_pos = camera_T[0:3, 3]

            chair_cloud = chair_mesh.sample_points_uniformly(number_of_points=1000)
            chair_T, rmse = icp_fit_object(chair_cloud, pcd_bbox, camera_pos)
            print(chair_T)
            print(rmse)

            cm = chair_mesh.copy()
            cm.transform(chair_T)
            o3d.visualization.draw_geometries([cm, pcd])
            


        # fit a plane for flat objects
        elif int(class_id) in FLAT:
            # oboxes = detect_planar_patches(pcd_bbox)
            _, obox, fit_rate = segment_plane(pcd_bbox)

            if fit_rate < MIN_FIT_RATE: 
                continue
            

            # compute the 4 corner points from planes obox 
            corners = obox_to_corners(obox).reshape((1, 4, 3))

            area = area_of_plane(corners)
            area_factor = min(area / AREA_NORMALIZATION, 1)

            # computing the distance to the most similar plane
            min_plane_dist = 10e10
            if planes.shape[0] > 0: 
                squared_diff = (planes - corners) ** 2
                sum_squared_diff = np.sum(np.sum(squared_diff, axis=1), axis=1)
                most_similar_idx = np.argmin(sum_squared_diff) 

                # having the same object class is required for similarity
                if plane_labels[most_similar_idx] == class_id: 
                    min_plane_dist = np.min(sum_squared_diff) / area_factor

            # update plane if it is similar and has the same class id 
            if min_plane_dist < MIN_PLANE_DISTANCE: 
                old = planes[most_similar_idx, : , :]
                k = PLANE_UPDATE_WEIGHT * fit_rate ** 2 * area_factor
                print("UPDATE WEIGHT: ", k)
                planes[most_similar_idx, : , :] = old * (1-k) + corners * k

            # else add new plane
            else:
                planes = np.vstack([planes, corners])
                plane_labels = np.vstack([plane_labels, np.array(class_id)])


        # send planes to unity 
        send_planes(np.copy(planes), np.copy(plane_labels)) # copy numpy array to avoid memory corruption because of synchronization

    # timing of the plane extraction pipeline
    end_time = time.time()
    elapsed_time = end_time - start_time
    rospy.loginfo(f"Plane extraction elapsed time: {elapsed_time} seconds")
    


def area_of_plane(corners):
    A = corners[0, 1,:] - corners[0, 0,:]
    B = corners[0, 2,:] - corners[0, 0,:]
    area = np.linalg.norm(np.cross(A, B))
    return area

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


