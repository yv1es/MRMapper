#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import mxnet as mx 
from gluoncv import model_zoo, data
import threading
import numpy as np
import cv2
import time
from scipy.spatial.transform import Rotation

from unity_sender import UnitySender 
from detections import ObjectManager, ICPObject
from point_cloud_processing import *
from utils import *
import constants


"""
This ROS node periodically caputres a tiplet consisting of a camera frame, camera transform and point cloud.  
It then uses YoloV3 to predict boudning boxes of objects in the camera frame. It then projects these 2D bounding 
boxes into a 3D frustum in the point cloud. Then it tries to fit a plane or a mesh to the frustum point cloud. 
It keeps track of good fitting planes or objects and updates them with new detections. 
"""



# load yoloV3 model 
net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)

# globals
bridge = CvBridge()
chair_mesh = load_chair_mesh()
lock = threading.Lock()
capture = False

# store for captured messages
cloud_map_msg = None
odom_msg = None
image_msg = None




def capture_callback(event):
    """
    This callback function in periodically exectued and its purpose it 
    to periodically start the sense making pipeline.
    Adjust the frequency of its execution with FREQ_PLANE_EXTRACTION constant. 
    It guarantees that no captures run concurrently with locking. 

    Args:
        event: Timer event (unused) 

    Returns:
        None
    """

    # Start capture of camera frame, camera transform and point cloud
    # locking prevents multiple captures running in parallel
    if not lock.acquire(blocking=False):
        return
    else:
        try:
            make_capture()
        finally:
            lock.release()


def make_capture():
    """
    After this function is called it, captures the first camera frame, camera transform and point cloud that is published after the call. 
    It converts the caputred ROS messages into OpenCv, numpy and Open3d typed objects and calls process_capture with the converted triplet.  

    Args:
        None

    Returns:
        None
    """
    global capture, cloud_map_msg, odom_msg, image_msg, lock
    capture = False
    
    # delete stored triplet
    odom_msg = None
    image_msg = None
    cloud_map_msg = None
    
    log("Starting capture")

    capture = True
    while cloud_map_msg is None or odom_msg is None or image_msg is None:
        if rospy.is_shutdown():
            return 
        else:
            continue # wait until triplet is complete

    # convert from ROS to open3d point cloud 
    pcd = ros_pointcloud_to_o3d(cloud_map_msg)

    # convert from ROS to opencv image
    img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # make the camera transform matrix 
    camear_transform = transform_matrix_from_odom(odom_msg)

    log("Captured camera frame, camera transform, point cloud tripplet")
    process_capture(img, camear_transform, pcd)



# callbacks for ROS subscription    
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




def process_capture(img, camera_transform, pcd):
    """
    This function predicts bounding boxes of objects in the camera frame. 
    It projects the 2D bounding boxes into a 3D frustum point cloud. 
    It then fits a plane or mesh to the frustum point cloud. 
    When a good fitting plane [or icp_object] is found it updates already found 
    planes [icp_objects] that are close or it adds them as a new plane [icp_object]

    Args:
        img: Captured camera frame
        camera_transform: Camera transform matrix
        pcd: Global point cloud

    Returns:
        None
    """
    start_time = time.time()
    
    # yolo object detection
    class_ids, confidences, boxes = yolo_predict(img)
    print_yolo_prediction(class_ids, confidences, boxes)
    
    # loop over detected boxes 
    for i, (class_id, confidence, bbox) in enumerate(zip(class_ids, confidences, boxes)):

        # only process detections for which we can fit a plane or icp 
        if int(class_id) not in constants.ICP_OBJECTS | constants.FLAT_OBJECTS: 
            continue

        log("--------------------------------------")
        log("Detection {}: Starting processing".format(i))

        # only proceed when the bounding box has MIN_BOUNDING_BOX_MARGIN pixels distance from the frame borders
        x1, y1, x2, y2 = bbox
        if min(x1, y1) < constants.MIN_BOUNDING_BOX_MARGIN or min(img.shape[1] - x2, img.shape[0] - y2) < constants.MIN_BOUNDING_BOX_MARGIN: 
            log("Detection {}: detection was to close to frame borders".format(i))
            continue

        # project bounding box frustum into 3D and make a point cloud with all points inside
        pcd_bbox = pcd_from_bbox(bbox, camera_transform, pcd)

        # only proceed when the cut out point cloud has enough points
        if  len(pcd_bbox.points) < 10:
            log("Detection {}: not enough points in frustum point cloud".format(i))
            continue 

        # remove points that should be hidden from the camera
        camera_pos = camera_transform[0:3, 3]
        pcd_bbox = frustum_hidden_point_removal(pcd_bbox, camera_pos)
        pcd_bbox = keep_largest_cluster(pcd_bbox)

        # only proceed when the cut out point cloud has enough points
        if  len(pcd_bbox.points) < 30:
            log("Detection {}: not enough visible points in frustum point cloud".format(i))
            continue 
    
        
        # compute icp transform for specific objects
        if int(class_id) in constants.ICP_OBJECTS: 
            if confidence < 0.8:
                log("Detection {}: Not confident enough".format(i))
                continue
            log("Detection {}: Starting icp fitting".format(i))
            source_cloud = chair_mesh.sample_points_uniformly(number_of_points=1000)
            new_icp_object = icp_fit_object(source_cloud, pcd_bbox, camera_pos, class_id)
            log("Detection {}: fitted icp object with rmse={} quality={}".format(i, new_icp_object._rmse, new_icp_object.quality))
            icp_object_manager.add(new_icp_object)
            
        
        # fit a plane for flat objects
        if int(class_id) in constants.FLAT_OBJECTS:
            log("Detection {}: Starting plane fitting".format(i))
            # plane fitting
            new_plane = fit_plane(pcd_bbox, class_id)
            # new_plane might be none
            if new_plane:     
                log("Detection {}: fitted plane with fit_rate={} (quality={})".format(i, new_plane._fit_rate, new_plane.quality))
                plane_manager.add(new_plane)
                    
    
    # send_planes 
    plane_data = plane_manager.get_all_bytes()
    plane_sender.send(plane_data)

    # send icp_objects
    icp_object_data = icp_object_manager.get_all_bytes()
    icp_object_sender.send(icp_object_data)


    # timing the sense making pipeline
    end_time = time.time()
    elapsed_time = end_time - start_time
    log("Processed captured tripplet in {} seconds".format(elapsed_time))
    log("==================================================================")


def yolo_predict(img):
    """
    Perform object detection using YOLO on the given image.

    Args:
        img (numpy.ndarray): Input image for object detection.

    Returns:
        tuple: A tuple containing class IDs, confidences, and bounding boxes of the detected objects.
    """

    # prepare image for yolo
    x, tmp_img = data.transforms.presets.yolo.transform_test(mx.nd.array(img), short=512)

    # compute scale factors of the transform above
    scale = (np.array(img.shape) / np.array(tmp_img.shape))[:2]

    # predict bounding boxes with yolo
    class_ids, confidences, boxes = net(x)

    # convert from MXnet arrays to numpy arrays with reasonable shape
    class_ids = class_ids.asnumpy().reshape(100, 1).astype(np.int32)
    confidences = confidences.asnumpy().reshape(100, 1)
    boxes = boxes.asnumpy().reshape(100, 4)

    # undo the yolo input transformation such that the bounding boxes align with the original camera frame 
    boxes[:, 0] *= scale[0]
    boxes[:, 2] *= scale[0]
    boxes[:, 1] *= scale[1]
    boxes[:, 3] *= scale[1]

    # filter predictions by confidence
    idx = np.where(confidences < constants.YOLO_CONFIDENCE_TRESHOLD)[0][0]
    class_ids = class_ids[:idx]
    confidences = confidences[:idx]
    boxes = boxes[:idx]
    return class_ids, confidences, boxes


def print_yolo_prediction(class_ids, confidences, boxes):
    """
    Print the objects detected by YOLO.

    Args:
        class_ids (numpy.ndarray): Array of class IDs.
        confidences (numpy.ndarray): Array of confidences.
        boxes (numpy.ndarray): Array of bounding boxes.
    
    Returns:
        None
    """

    log("Yolo detected {} objects:".format(len(class_ids)))
    for i in range(len(class_ids)):
        class_id = class_ids[i]
        confidence = confidences[i]
        bbox = boxes[i]
        log("\t Detection {}:, Class={}, Confidence={}".format(i, constants.CLASSES[int(class_id)], confidence))


def log(s : str) -> None:
    rospy.loginfo("[Sense making] " + s)


def main():
    # setup plane and object manager 
    global plane_manager, icp_object_manager 
    plane_manager = ObjectManager(constants.MIN_PLANE_DISTANCE, log)
    icp_object_manager = ObjectManager(constants.MIN_ICP_OBJ_DIST, log)

    # setup unity senders
    global plane_sender, icp_object_sender
    plane_sender = UnitySender(constants.HOST, constants.PORT_PLANES, 'Plane Sender')
    icp_object_sender = UnitySender(constants.HOST, constants.PORT_OBJECTS, 'Object Sender')
    plane_sender.start()
    icp_object_sender.start()


    # subscribe to topics
    rospy.init_node('sense_making', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)

    # set timer for capture callback 
    rospy.Timer(rospy.Duration(constants.FREQ_SENSE_MAKING), capture_callback)
    rospy.on_shutdown(shutdown)
    rospy.spin()


def shutdown():
    plane_sender.stop()
    icp_object_sender.stop()
    print("sense_making shutdown")




if __name__ == '__main__':
    try:
        main()
    finally:
        shutdown()


