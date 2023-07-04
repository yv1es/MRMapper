import pyrealsense2 as rs
import socket as s
import numpy as np 
import cv2
import pickle
import struct
import time

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60
HOST = s.gethostname() 
PORT = 5000 

"""  
This script starts a connected Intel RealSense camera and streams the RGBD camera feed into the docker container. 
The camera_publisher ROS node in the container receives the RGBD stram and publishes it to rtabmap_ros and MRMapper.  
"""


def setupSocket():
    sock = s.socket(s.AF_INET, s.SOCK_STREAM)
    sock.connect((HOST, PORT))  
    return sock


def setup_realsense():
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
    
    return pipeline
 

class Msg():
    def __init__(self, color, depth) -> None:
        self.color = color
        self.depth = depth


def encode(color, depth):
    # compress color 
    _, color = cv2.imencode('.jpg', color, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    msg = Msg(color, depth)
    msg_bytes = pickle.dumps(msg)
    msg_bytes = struct.pack('!I', len(msg_bytes)) + msg_bytes
    return msg_bytes


def main():
    print("Setting up RealSense")
    pipeline = setup_realsense()

    # setup aligner
    align_to = rs.stream.color
    aligner = rs.align(align_to)
    

    while True:
        socket = setupSocket()
        try:
            while True:
                frames = pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = aligner.process(frames)

                # Get aligned frames
                depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not depth_frame or not color_frame:
                    continue
                
                # encode frames into one numpy array 
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                data = encode(color_image, depth_image)
                socket.send(data)
                

        except ConnectionResetError as _:
            time.sleep(1)
        except ConnectionAbortedError as _:
            print("Attempting to connect...")
            time.sleep(1)
        except KeyboardInterrupt as _:
            break
        
    print("Stopping capture")
    socket.close()
    pipeline.stop()
    print("Exiting")
    



if __name__=='__main__':
    main()


