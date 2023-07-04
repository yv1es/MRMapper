import argparse 
import pyrealsense2 as rs
import numpy as np
from datetime import datetime
import cv2
import os 

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60

"""
This script records a depth and color video and saves it as two avi files. 
The recording starts when the sript is run and a RealSense camera is connected. 
Press q to end the recording and write the output files. 
"""


def setup_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default=datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), help="output folder (default: current date and time)")
    args = vars(ap.parse_args())

    # Create output folder if it doesn't exist
    output_folder = args["output"]
    os.makedirs(output_folder, exist_ok=False)

    # Set the output file paths
    color_file = os.path.join(output_folder, 'color.avi')
    depth_file = os.path.join(output_folder, 'depth.avi')

    # Configure depth and color streams
    print("Starting RealSense")
    pipeline = setup_realsense()

    # Create a VideoWriter object to save the footage
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    color_out = cv2.VideoWriter(color_file, fourcc, FPS, (FRAME_WIDTH, FRAME_HEIGHT))
    depth_out = cv2.VideoWriter(depth_file, fourcc, FPS, (FRAME_WIDTH, FRAME_HEIGHT), isColor=False)

    # setup aligner
    align_to = rs.stream.color
    aligner = rs.align(align_to)

    print("Recording started")
    try:
        while True:
            # Wait for the next frameset
            frames = pipeline.wait_for_frames()
            
            # Align the depth frame to color frame
            aligned_frames = aligner.process(frames)

            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert the color and depth frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Write the color and depth frames to the respective video files
            color_out.write(color_image)
            depth_out.write(depth_image)

            # Display the color frame
            cv2.imshow('RealSense Camera', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping recording")
                break

    finally:
        # Release the VideoWriter and pipeline resources
        color_out.release()
        depth_out.release()
        pipeline.stop()

        # Close the OpenCV windows
        cv2.destroyAllWindows()
        print("Files written to \n \t-{} \n \t-{}".format(color_file, depth_file))



if __name__=='__main__':
    main()
