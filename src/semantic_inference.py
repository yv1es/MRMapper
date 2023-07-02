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
from point_cloud_processing import *
from utils import *
from constants import *


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
    to periodically start the semantic inference pipeline.
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
        if int(class_id) not in ICP_OBJECTS | FLAT_OBJECTS: 
            continue

        log("------------------------------------------")
        log("Detection {}: Starting processing".format(i))

        # only proceed when the bounding box has MIN_BOUNDING_BOX_MARGIN pixels distance from the frame borders
        x1, y1, x2, y2 = bbox
        if min(x1, y1) < MIN_BOUNDING_BOX_MARGIN or min(img.shape[1] - x2, img.shape[0] - y2) < MIN_BOUNDING_BOX_MARGIN: 
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
        if  len(pcd_bbox.points) < 10:
            log("Detection {}: not enough visible points in frustum point cloud".format(i))
            continue 
    
        # compute icp transform for specific objects
        if int(class_id) in ICP_OBJECTS: 
            log("Detection {}: Starting icp fitting".format(i))

            # iterative icp 
            source_cloud = chair_mesh.sample_points_uniformly(number_of_points=1000)
            icp_transform, rmse = icp_fit_object(source_cloud, pcd_bbox, camera_pos)
            
            log("Detection {}: found ICP transform with RMSE={}".format(i, rmse))
            # filter for low rmse 
            if rmse > MAX_RMSE: 
                log("Detection {}: the RMSE was too high".format(i))
                continue
            
            log("Detection {}: ICP object found".format(i))
            # create new object and add it to manager
            new_object = ICPObject(icp_transform, class_id, rmse)
            icp_object_manager.add(new_object)
            
        # fit a plane for flat objects
        if int(class_id) in FLAT_OBJECTS:
            log("Detection {}: Starting plane fitting".format(i))
            
            # plane segmantation
            obox, fit_rate = fit_plane(pcd_bbox)
            
            log("Detection {}: fitted plane with fit_rate={}".format(i, fit_rate))
            if fit_rate < MIN_FIT_RATE: 
                log("Detection {}: fit_rate was too low".format(i))
                continue
            
            log("Detection {}: Plane found".format(i))
            # compute the 4 corner points from planes obox 
            corners = obox_to_corners(obox).reshape((4, 3))
            new_plane = Plane(corners, class_id, fit_rate)
            plane_manager.add(new_plane)
    
    # send_planes 
    plane_data = plane_manager.get_all_bytes()
    plane_sender.send(plane_data)

    # send icp_objects
    icp_object_data = icp_object_manager.get_all_bytes()
    icp_object_sender.send(icp_object_data)


    # timing of the plane extraction pipeline
    end_time = time.time()
    elapsed_time = end_time - start_time
    log("Processed captured tripplet in {} seconds".format(elapsed_time))
    log("_______________________________________________________________")
    log("_______________________________________________________________")


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
    idx = np.where(confidences < YOLO_CONFIDENCE_TRESHOLD)[0][0]
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
        log("\t Detection {}:, Class={}, Confidence={}".format(i, CLASSES[int(class_id)], confidence))


def log(s : str) -> None:
    rospy.loginfo("[Semantic Inference] " + s)


def main():
    # setup plane and object manager 
    global plane_manager, icp_object_manager 
    plane_manager = ObjectManager(MIN_PLANE_DISTANCE)
    icp_object_manager = ObjectManager(MIN_ICP_OBJ_DIST)

    # setup unity senders
    global plane_sender, icp_object_sender
    plane_sender = UnitySender(HOST, PORT_PLANES, 'Plane Sender')
    icp_object_sender = UnitySender(HOST, PORT_OBJECTS, 'Object Sender')
    plane_sender.start()
    icp_object_sender.start()


    # subscribe to topics
    rospy.init_node('semantic_inference', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, cloud_map_callback)
    rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)

    # set timer for capture callback 
    rospy.Timer(rospy.Duration(FREQ_SEMANTIC_INFERENCE), capture_callback)
    rospy.spin()






class ObjectManager: 
    """
    A class for managing objects. The managed objects need to implement a distance, update and tobytes method. 

    Attributes:
        _min_dist (float): The minimum distance threshold for considering an object as new. 
        _objects (list): A list of objects.

    Methods:
        __init__(self, min_dist): Initializes the ObjectManager instance.
        add(self, object): Adds an object to the object manager.
        get_all(self): Returns all objects in the object manager.
        get_all_bytes(self): Returns all objects serialized as a byte string.
        _find_closest(self, object): Finds the closest object in the object manager to the given object.
    """

    def __init__(self, min_dist) -> None:
        self._min_dist = min_dist
        self._objects = []

    def add(self, object): 
        """
        Adds an object to the object manager.
        If a similar object is found within the minimum distance threshold, it updates the closest object.
        Otherwise, it adds the new object to the object manager.

        Args:
            object: The object to be added.
        
        Returns:
            None
        """
        closest, min_dist = self._find_closest(object)

        log("min. distance to closest similar object={}".format(min_dist))
        # update if the closest object is closer than the min_dist threshold
        if min_dist < self._min_dist: 
            log("Updating closest object")
            closest.update(object)
        else: 
            log("Adding new object")
            self._objects.append(object)

    def get_all(self): 
        return self._objects
    
    def get_all_bytes(self):
        """
        Returns all objects serialized as one byte string.

        Returns:
            bytes: Objects represented in bytes format.
        """
        b = b""
        for o in self._objects: 
            b += o.tobytes()
        return b 
     
    def _find_closest(self, object): 
        """
        Finds the closest object in the object manager to the given object.

        Args:
            object: The object for which to find the closest object.

        Returns:
            tuple: A tuple containing the closest object and the distance to it.
        """
        min_dist = INF
        closest = None
        for c in self._objects: 
            dist = object.distance(c)
            if dist < min_dist: 
                closest = c 
                min_dist = dist 
        return closest, min_dist


class Plane: 
    """
    A class representing a plane.

    Attributes:
        _corners (ndarray): The corners of the plane.
        _class_id (int): The class ID of the plane.
        _fit_rate (float): The fit rate of the plane.

    Methods:
        __init__(self, corners, class_id, fit_rate): Initializes the Plane instance.
        distance(self, other_plane): Calculates the distance between the plane and another plane.
        update(self, p): Updates the plane with another plane's information.
        area(self): Calculates the area of the plane.
        get_corners(self): Returns the corners of the plane.
        get_class_id(self): Returns the class ID of the plane.
        tobytes(self): Converts the plane object to bytes.
    """


    def __init__(self, corners, class_id, fit_rate): 
        """
        Initializes the Plane instance.

        Args:
            corners (ndarray): The corners of the plane.
            class_id (int): The class ID of the plane.
            fit_rate (float): The fit rate of the plane.
        
        Returns:
            None
        """
        self._corners = corners
        self._class_id = class_id    
        self._fit_rate = fit_rate
    
    
    def distance(self, other_plane):
        """
        Calculates the distance between the plane and another plane.

        If the class IDs of the planes don't match, returns infinity (INF).
        Otherwise, calculates the distance based on the area factor and squared difference of corners.

        Args:
            other_plane (Plane): The other plane to calculate the distance from.

        Returns:
            float: The distance between the planes.
        """
        if self._class_id != other_plane.get_class_id():
            return INF 
        area_factor = min(self.area() / AREA_NORMALIZATION, 1)
        sum_squared_diff = np.sum((self._corners - other_plane.get_corners()) ** 2)
        return sum_squared_diff / area_factor

    def update(self, p): 
        """
        Updates the plane with another plane's information.

        Calculates the update weight based on the area factor and fit rate of the plane.
        Updates the corners of the plane using the weighted average.

        Args:
            p (Plane): The plane to update from.

        Retruns:
            None
        """
        area_factor = min(self.area() / AREA_NORMALIZATION, 1)
        # k = PLANE_UPDATE_WEIGHT * area_factor * self._fit_rate ** 2
        k = PLANE_UPDATE_WEIGHT
        log("Plane update weight={}".format(k))
        self._corners = self._corners * (1-k) + p.get_corners() * k

    def area(self):
        """
        Calculates the area of the plane.

        Returns:
            float: The area of the plane.
        """
        A = self._corners[1,:] - self._corners[0,:]
        B = self._corners[2,:] - self._corners[0,:]
        area = np.linalg.norm(np.cross(A, B))
        return area
    
    def get_corners(self):
        return self._corners

    def get_class_id(self): 
        return self._class_id

    def tobytes(self): 
        """
        Converts the plane object to bytes.

        Returns:
            bytes: The plane represented in bytes.
        """
        b = self._corners.astype(np.float32).tobytes() + \
            np.array(self._class_id).astype(np.float32).tobytes()
        return b


class ICPObject:
    """
    A class representing an objec found by the Iterative Closest Point (ICP) algorithm.

    Attributes:
        _transform (ndarray): The transformation matrix of the object.
        _class_id (int): The class ID of the object.
        _rmse (float): The root mean square error (RMSE) from the ICP fit of the object.

    Methods:
        __init__(self, transform, class_id, rmse): Initializes the ICPObject instance.
        distance(self, o): Calculates the distance between the object and another object.
        update(self, o): Updates the object with another object's information.
        get_transform(self): Returns the transformation matrix of the object.
        get_class_id(self): Returns the class ID of the object.
        tobytes(self): Converts the object to bytes.
    """


    def __init__(self, transform, class_id, rmse):
        """
        Initializes the ICPObject instance.

        Args:
            transform (ndarray): The transformation matrix of the object.
            class_id (int): The class ID of the object.
            rmse (float): The root mean square error (RMSE) of the object.
        """
        self._transform = transform 
        self._class_id = class_id 
        self._rmse = rmse

    def distance(self, new_object): 
        """
        Calculates the distance between the object and another object.

        If the class IDs of the objects don't match, returns infinity (INF).
        Otherwise, calculates the distance based on the squared difference of transforms.

        Args:
            new_object (ICPObject): The other object to calculate the distance from.

        Returns:
            float: The distance between the objects.
        """
        if self._class_id != new_object.get_class_id():
            return INF 
        sum_squared_diff = np.sum((self._transform - new_object.get_transform()) ** 2)
        return sum_squared_diff

    def update(self, new_object): 
        """
        Updates the object with another object's information.

        Applies a weighted average update to the transformation matrix of the object.

        Args:
            new_object (ICPObject): The object to update from.
        """
        k = ICP_OBJECT_UPDATE_WEIGHT
        self._transform = self._transform * (1-k) + new_object.get_transform() * k 
    
    def get_transform(self): 
        return self._transform

    def get_class_id(self): 
        return self._class_id

    def tobytes(self): 
        """
        Converts the object to bytes.

        Computes the translation and quaternion from the transformation matrix,
        and converts them along with the class ID to bytes.

        Returns:
            bytes: The object represented in bytes.
        """
        translation = self._transform[:3, 3]
        rotation_matrix = self._transform[:3, :3]
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        quaternion = quaternion / np.linalg.norm(quaternion)

        b = translation.astype(np.float32).tobytes() + \
            quaternion.astype(np.float32).tobytes() + \
            np.array([self._class_id]).astype(np.float32).tobytes()
        return b



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        plane_sender.stop()
        icp_object_sender.stop()


