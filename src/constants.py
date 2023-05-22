import socket

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

HOST = socket.gethostname()

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60

# Distorion
CAMERA_DISTORTION_MODEL = "plumb_bob"
CAMERA_D =  [0.0, 0.0, 0.0, 0.0, 0.0]

# intrinsics
CAMERA_K =  [618.3,    0.0,  316.2,
               0.0,  617.9,  242.3,
               0.0,    0.0,   1.0]   
 

# ports
PORT_CAMERA = 5000 
PORT_PCL = 5001
PORT_PLANES = 5003
PORT_ODOM = 5002

FREQ_PLANE_EXTRACTION = 1

# distance in pixels a bounding box needs to have from the frame border to be considered
MIN_BOUNDING_BOX_MARGIN = 5

MIN_PLANE_DISTANCE = 7.5

PLANE_UPDATE_WEIGHT = 0.1

# class ids which are flat and hence fit a plane
FLAT = {59, 60, 66, 67, 62}

# lables for the yolo class ids
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