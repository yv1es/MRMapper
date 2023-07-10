import numpy as np 
import cv2
import argparse
from scipy.spatial.transform import Rotation as R
from collections import defaultdict


from utils import VideoReader
from camera_constants import *

"""  
Takes a saved color-depth stream and computes the position of aruco markers.  
"""
 

# The camera position and rotation relative to the origin in Unity coordinate system (metric left-handed y-up)
CAMERA_POS = np.array([0.6, -0.135, 0])
CAMERA_ROT = np.array([15, -60, 0])



# Intrinsics and distortion
CAMERA_D =  np.array(CAMERA_D)
CAMERA_K =  np.array(CAMERA_K).reshape(3, 3)

# markers werer generated with https://chev.me/arucogen/
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_100
MARKER_SIZE = 100  # in mm



def pose_esitmation(frame, matrix_coefficients=CAMERA_K, distortion_coefficients=CAMERA_D):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_TYPE)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)
    
    d = {}
    # If markers are detected
    if len(corners) > 0:
        
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE/1000, matrix_coefficients, distortion_coefficients)
            
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, MARKER_SIZE/1000)  
            
            unity_tvec = tvec[0][0].copy()
            unity_tvec[1] = -unity_tvec[1]

            # convert from cammera coordinate system to world coordinates 
            unity_tvec = R.from_euler('XYZ', CAMERA_ROT, degrees=True).apply(unity_tvec) + CAMERA_POS
            id = ids[i][0]
            d[id] = unity_tvec
            print("ID: {}    POS: {}".format(id, unity_tvec))


    return d, frame





def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="input file")
    ap.add_argument("-fn", "--frame-number", required=False, type=int, help="only compute the marker positions for this frame")
    ap.add_argument("-a", "--average", action='store_true', help="average the positions over all frames")
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

    messurements = defaultdict(list)

    try:
        while True:

            color_image, _ = vid.nextFrame()                    
                            
            m, color_image_annotated = pose_esitmation(color_image)
            for id, pos in m.items(): 
                messurements[id].append(pos)

            # Display the color frame
            cv2.imshow('RealSense Camera', cv2.cvtColor(color_image_annotated, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping playback")
                raise Exception

    except Exception as e:
        if str(e):
            print(e)
    
    if args['average']:
        print('\n===================================================\n')
        print('Camera position: ', CAMERA_POS)
        print('Camera rotation: ', CAMERA_ROT)
        
        for id, poses in messurements.items(): 
            poses = np.vstack(poses)
            mean = np.mean(poses, axis=0)
            stdev = np.std(poses, axis=0)
            print('\n---------------------------------------------------\n')
            print("Marker id: ", id)
            print("\tMean position: ", mean)
            print("\tStandard dev.: ", stdev)
        print('\n---------------------------------------------------\n')
if __name__=='__main__':
    main()
    cv2.destroyAllWindows()

