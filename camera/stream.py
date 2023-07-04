import pyrealsense2 as rs
import socket as s
import numpy as np 
import time

from camera_constants import * 
from utils import setupRealsense, sendFrames

# Constants
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


def main():
    print("Setting up RealSense")
    pipeline = setupRealsense()

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
                sendFrames(color_image, depth_image)
                

        except ConnectionResetError as _:
            time.sleep(1)
        except ConnectionAbortedError as _:
            print("Attempting to connect...")
            time.sleep(1)
        except KeyboardInterrupt as _:
            break
        
    print("Stopping stream")
    socket.close()
    pipeline.stop()
    print("Exiting")
    



if __name__=='__main__':
    main()


