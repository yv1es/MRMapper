import argparse
from datetime import datetime
import pyrealsense2 as rs
import socket as s
import numpy as np 
import cv2

from utils import setupRealsense, encode
from camera_constants import * 


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default=datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), help="output file (default: current date and time)")
    args = vars(ap.parse_args())

    output_file = args["output"] + '.bin'

    print("Setting up RealSense")
    pipeline = setupRealsense()

    # setup aligner
    align_to = rs.stream.color
    aligner = rs.align(align_to)
    
    
    with open(output_file, 'ab') as file:
        while True:
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = aligner.process(frames)

            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame() # depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not depth_frame or not color_frame:
                continue
            
            # encode frames into one numpy array 
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            data = encode(color_image, depth_image)
            file.write(data)

            # Display the color frame
            
            cv2.imshow('RealSense Camera', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping recording")
                break

     
    pipeline.stop()
    cv2.destroyAllWindows()
    print("File written to \n \t-{}".format(output_file))

    
if __name__=='__main__':
    main()


