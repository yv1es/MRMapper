#!/usr/bin/env python3
import socket

# Camera parameters 
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Distorion
CAMERA_DISTORTION_MODEL = "plumb_bob"
CAMERA_D =  [0.0, 0.0, 0.0, 0.0, 0.0]

# Intrinsics
CAMERA_K =  [618.3,    0.0,  316.2,
               0.0,  617.9,  242.3,
               0.0,    0.0,   1.0]   


# Ports
HOST = socket.gethostname()
PORT_CAMERA = 5000 
PORT_PCL = 5001
PORT_ODOM = 5002
PORT_PLANES = 5003
PORT_OBJECTS = 5004


FREQ_SENSE_MAKING = 1  # how frequently the semantic inference is launched (in seconds)
FRUSTUM_DEPTH = 8 # how deep the frustum from the camera reaches into the scene (in meters)


# filtering 
YOLO_CONFIDENCE_TRESHOLD = 0.3  # minimum confidence required from yolo detections 
MIN_BOUNDING_BOX_MARGIN = 10  # minimum distance that a bounding box needs to have from the borders of the camera frame
DBSCAN_EPS = 0.12
DBSCAN_MIN_POINTS = 4 


# planes
MIN_PLANE_DISTANCE = 30   # when a plane has a distance > MIN_PLANE_DIST to all other planes then it is added as a new plane
PLANE_UPDATE_WEIGHT = 0.1      # update weigth a new detected plane(distance smaller than MIN_PLANE_DIST) has on the existing closest plane
FIT_RATE_NORMALIZATION = 0.07  # fit rates are mapped from [0, FIT_RATE_NORMALIZATION] to [0, 1] linearly, fit rates > FIT_RATE_NORMALIZATION are capped to 1 
AREA_NORMALIZATION = 2  # areas are mapped from [0, AREA_NORMALIZATION] to [0, 1] linearly, areas > AREA_NORMALIZATION are capped to 1 


# icp fitting
MIN_ICP_OBJ_DIST = 1   # when a icp_object has a distance > MIN_ICP_OBJ_DIST to all other icp objects then it is added as a new icp object
ICP_OBJECT_UPDATE_WEIGHT = 0.2  # update weigth a new detection of the same object (distance smaller than MIN_ICP_OBJ_DIST) has on the existing object
RMSE_NORMALIZATION = 0.099  # RMSE are mapped from [0, RMSE_NORMALIZATION] to [0, 1] linearly, rmse > RMSE_NORMALIZATION are capped to 1 


# minimum quality for objects
MIN_QUALITY = 0.25


# class ids which are flat and hence MRMapper should fit a plane
FLAT_OBJECTS = {59, 60, 66, 67, 73}


# class ids for which MRMapper should fit a icp_object (currently only the office chair is supported)
ICP_OBJECTS = {56}

CHAIR_MESH_PATH = "/root/catkin_ws/src/MRMapper/src/meshes/56.obj"

# map from class_id to class label





CLASSES = [ 'person',
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