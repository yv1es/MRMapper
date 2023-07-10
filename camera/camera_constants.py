import socket as s

# Camera parameters 
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60

# Distorion
CAMERA_DISTORTION_MODEL = "plumb_bob"
CAMERA_D =  [0.0, 0.0, 0.0, 0.0, 0.0]

# Intrinsics
CAMERA_K =  [618.3,    0.0,  316.2,
               0.0,  617.9,  242.3,
               0.0,    0.0,   1.0]   

HOST = s.gethostname() 
PORT_CAMERA = 5000 