import socket as s
import cv2
import time
import argparse

from utils import VideoReader, sendImages
from camera_constants import *


"""  
This script streams a recorded recorded video as if a camera was connected. 
"""


def setupSocket():
    sock = s.socket(s.AF_INET, s.SOCK_STREAM)
    sock.connect((HOST, PORT_CAMERA))  
    return sock

 

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="input file")
    ap.add_argument("-ns", "--no_stream", action='store_true', help="do not start a stream to MRMapper")
    ap.add_argument("-fn", "--frame_number", type=int, default=0, required=False, help="the number of the first frame")
    args = vars(ap.parse_args())

    input_file_path = args['input']
    vid = VideoReader(input_file_path)    
    vid.skipFrames(max(0, args['frame_number']))

    while True:
             
        try:
            if not args['no_stream']:
                socket = setupSocket()
            
            while True:                                    
                color_image, depth_image = vid.nextFrame()
                if not args['no_stream']:
                    sendImages(color_image, depth_image, socket)
                
                # Display the color frame
                cv2.imshow('RealSense Camera', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Stopping playback")
                    raise Exception

                
        except ConnectionResetError as _:
            time.sleep(1)
        except ConnectionAbortedError as _:
            print("Attempting to connect...")
            time.sleep(1)
        except Exception as e:
            if str(e):
                print(e)
            break
        
    if not args['no_stream']:
        socket.close()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()


