import os 
import cv2
import struct
import pickle
import numpy as np  
import pyrealsense2 as rs
from camera_constants import *

def setupRealsense():
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("Requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.rgb8, FPS)
    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT,  rs.format.z16, FPS)

    cfg = pipeline.start(config)
    profile = cfg.get_stream(rs.stream.color)  # Fetch stream profile for depth stream
    intr = profile.as_video_stream_profile().get_intrinsics()
    print(intr)
    print(f"fx: {intr.fx} fy: {intr.fy} ppx: {intr.ppx} ppy: {intr.ppy}")
    print("Distortion model:", intr.model)
    print("Distortion coefficients:", intr.coeffs)
    
    return pipeline


def sendImages(color_image, depth_image, socket): 
    socket.send(encode(color_image, depth_image))


def encode(color, depth):
    # compress color 
    _, color = cv2.imencode('.jpg', color, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    msg = Msg(color, depth)
    msg_bytes = pickle.dumps(msg)
    msg_bytes = struct.pack('!I', len(msg_bytes)) + msg_bytes
    return msg_bytes


def decode(msg_bytes):
    msg = pickle.loads(msg_bytes)
    color = cv2.imdecode(np.frombuffer(msg.color, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth = msg.depth 
    return color, depth


class Msg():
    def __init__(self, color, depth) -> None:
        self.color = color
        self.depth = depth


class VideoReader:
    def __init__(self, path) -> None:
        if not os.path.exists(path):
            raise Exception("File path does not exist")

        self.file = open(path, 'rb') 

    def __del__(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()

    def nextFrame(self):
        # Receive the size of the data and then the data itself from the socket connection
        data_size = self.file.read(4)
        if not data_size:
            raise Exception("Video ended")
        
        size = struct.unpack('!I', data_size)[0]
        data = self.file.read(size)

        color_image, depth_image = decode(data)
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        return color_image, depth_image

    def skipFrames(self, n):
        for i in range(n):
            self.nextFrame()
