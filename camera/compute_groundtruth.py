import pyrealsense2 as rs
import socket as s
import numpy as np 
import cv2
import pickle
import struct
import time
import argparse
import os

from utils import VideoReader
from camera_constants import *

"""  
Takes a saved color-depth stream and computes the position of aruco markers.  
"""
 
CAMERA_D =  np.array(CAMERA_D)

# Intrinsics
CAMERA_K =  np.array(CAMERA_K).reshape(3, 3)



def pose_esitmation(frame, aruco_dict_type=cv2.aruco.DICT_5X5_50, matrix_coefficients=CAMERA_K, distortion_coefficients=CAMERA_D):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.02)  

            print("ID: {}    POS: {}".format(ids[i], tvec))

    return frame



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="input file")
    ap.add_argument("-fn", "--frame-number", required=False, type=int, help="only compute the marker positions for this frame")
    args = vars(ap.parse_args())

    input_file_path  = args['input']
    vid = VideoReader(input_file_path)

    if args['frame_number']:
        vid.skipFrames(max(0, args['frame_number']))
        color_image, _ = vid.nextFrame()
        pose_esitmation(color_image)
        cv2.imshow('RealSense Camera', color_image)
        cv2.waitKey(0)
        return

    while True:
        try:
            while True:

                color_image, _ = vid.nextFrame()                    
                                
                color_image = pose_esitmation(color_image)
                
                # Display the color frame
                cv2.imshow('RealSense Camera', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Stopping playback")
                    raise Exception
 
        except Exception as e:
            if str(e):
                print(e)
            break

    

if __name__=='__main__':
    main()
    cv2.destroyAllWindows()

