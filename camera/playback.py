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
    msg_bytes = struct.pack('!I', len(msg_bytes)) + msg_bytes
    return msg_bytes


def decode(msg_bytes):
    msg = pickle.loads(msg_bytes)
    color = cv2.imdecode(np.frombuffer(msg.color, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth = msg.depth 
    return color, depth


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="input file")
    args = vars(ap.parse_args())

    input_file = args['input']

    if not os.path.exists(input_file):
        print("Input file not found! Exiting...")
        exit(0)

    with open(input_file, 'rb') as file:
        
        while True:
            socket = setupSocket()
            try:
                while True:
                                        
                    # Receive the size of the data and then the data itself from the socket connection
                    data_size = file.read(4)
                    if not data_size:
                        print("Video ended")
                        raise Exception
                    
                    size = struct.unpack('!I', data_size)[0]
                    data = file.read(size)

                    color_image, depth_image = decode(data)
                    
                    # Display the color frame
                    cv2.imshow('RealSense Camera', color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Stopping playback")
                        raise Exception

                
                    socket.send(encode(color_image, depth_image))
                    
            except ConnectionResetError as _:
                time.sleep(1)
            except ConnectionAbortedError as _:
                print("Attempting to connect...")
                time.sleep(1)
            except Exception as _:
                break
        
    print("Stopping playback")
    socket.close()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()


