import pyrealsense2 as rs
import socket as s
import numpy as np 
import cv2
import pickle
import struct
import time
import argparse
import os

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60
HOST = s.gethostname() 
PORT = 5000 

"""  
This takes a folder containing a depth and a color video. It streams the recorded video as if a camera was connected. 
"""


def setupSocket():
    sock = s.socket(s.AF_INET, s.SOCK_STREAM)
    sock.connect((HOST, PORT))  
    return sock

 

class Msg():
    def __init__(self, color, depth) -> None:
        self.color = color
        self.depth = depth


def encode(color, depth):
    # compress color 
    _, color = cv2.imencode('.jpg', color, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    msg = Msg(color, depth)
    msg_bytes = pickle.dumps(msg)
    return msg_bytes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", help="input folder containing the color and depth files")
    args = vars(ap.parse_args())

    # Create output folder if it doesn't exist
    output_folder = args["input"]
    
    # Set the output file paths
    color_file = os.path.join(output_folder, 'color.avi')
    depth_file = os.path.join(output_folder, 'depth.avi')

    if not os.path.exists(color_file):
        print("Color file not found! Exiting...")
        exit(0)

    if not os.path.exists(depth_file):
        print("Depth file not found! Exiting...")
        exit(0)


    color_cap = cv2.VideoCapture(color_file)
    depth_cap = cv2.VideoCapture(depth_file)

    while True:
        socket = setupSocket()
        try:
            while True:

                # read next frames
                ret_color, color_frame = color_cap.read()
                ret_depth, depth_frame = depth_cap.read()

                # Validate that both frames are valid
                if not ret_color or not ret_depth:
                    print("Video ended")
                    break
                
                # encode frames into one numpy array 
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                data = encode(color_image, depth_image)
                
                # Send the size of the data and then the data itself over the socket connection
                size = len(data)
                socket.send(struct.pack('!I', size))
                socket.send(data)

                # Display the color frame
                cv2.imshow("Color", color_frame)

                # Display the depth frame
                cv2.imshow("Depth", depth_frame)

                # Exit the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except ConnectionResetError as _:
            print("Attempting to connect...")
            time.sleep(1)
        except ConnectionAbortedError as _:
            print("Attempting to connect...")
            time.sleep(1)
        except KeyboardInterrupt as _:
            break
        
    print("Stopping playback")
    socket.close()

    # Release the video file resources
    color_cap.release()
    depth_cap.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()


