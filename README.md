# MRMapper - Detection and Sense Making for Mixed Reality

## Installation
### Prerequisites
Before installing MRMapper, ensure that your system meets the following requirements:
* Windows operating system
* [Docker](https://www.docker.com/) installed
* [Unity](https://unity.com) installed
* [Python 3](https://www.python.org/) with [virtualenv](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/) installed

### Core
To set up the core functionality of MRMapper, follow these steps:

1. Clone this repository to your local machine.
2. Open a terminal and navigate to the `MRMapper/core` folder.
3. Execute the provided batch script: `build.bat` to build the Docker image named `mrmapper-image`. Please note that the image will require approximately 13 GB of disk space, and the build process can take up to an hour.

### Camera Control
The camera control scripts, responsible for managing the RealSense camera, run directly on the host system. To set up the necessary environment and install dependencies, follow these steps:

1. Open a PowerShell window and navigate to the `MRMapper/camera` folder.
2. Create a new virtual environment named "env" by executing the following command: `python -m venv env`.
3. Activate the virtual environment with the command: `.\env\Scripts\activate`. Note: If you encounter an error related to script execution policy, run the following command in an administrator PowerShell to allow execution of locally created scripts: `set-executionpolicy remotesigned`.
4. Install the required dependencies inside the virtual environment using the command: `pip install -r requirements.txt`.

Congratulations! The setup process is now complete, and MRMapper is ready to be used.

## Running MRMapper

MRMapper consists of three main components:

### Camera
The system has been tested with an [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) camera connected via a USB 3.0 cable.

To run the camera scripts, please ensure that the virtual environment set up during the installation is active in your current terminal session. If you are using PowerShell, "(env)" should appear to the left of "PS". If not, activate the virtual environment by executing the script `/MRMapper/camera/env/Scripts/activate`.

In the `MRMapper/camera` folder, you will find several Python scripts:

* `stream.py`: Start a live stream from the camera to MRMapper for real-time operation.
* `record.py`: Record footage with the camera to a file on disk. Use the "-h" flag for help on how to use this script.
* `playback.py`: View recorded footage and replay it for MRMapper. This is useful for testing and development. Use the "-h" flag for help on how to use this script.
* `groundtruth.py`: Determine the position of ArUco markers in the scene relative to the current camera position. This is useful for ground truth evaluation. Use the "-h" flag for help on how to use this script.

### Core
To start the MRMapper core, navigate to `MRMapper/core` and execute the `run.bat` script. This will launch a Docker container with MRMapper's core functionality.

### Unity
The Unity project for MRMapper can be found under `MRMapper/unity`. Open Unity and launch the play view.

The primary logic is contained under the "MRMapper" game object.

After playing the Unity scene, it will connect to the core running in the container. It will spawn a child game object representing the reconstructed point cloud and other game objects representing fitted planes and objects.

### Summary
To run MRMapper, follow these steps:

1. Launch the MRMapper core with `MRMapper/core/run.bat`.
2. Start Unity and click play to enter the play view.
3. To view a prerecorded file from disk, use `MRMapper/camera/playback.py`. Alternatively, connect a RealSense camera and start the stream with `MRMapper/camera/stream.py`.

You should now see the point cloud and reconstructed planes in Unity.

## Project Hierarchy

The MRMapper project is organized into the following directories:

* `MRMapper/core/ros_node/mr-mapper/src/`: Contains the Python source code for the core functionalities, including object detection and sense-making.
* `MRMapper/camera/`: Includes all the scripts for recording and streaming with the RealSense camera.
* `MRMapper/unity/MRMapper`: Contains the Unity demo project.

## Remarks

### Camera Calibration
The `MRMapper/camera/stream.py` script will print the camera calibration of the connected RealSense camera. It is essential to make sure you update the camera parameters specific to your RealSense device. Update the parameters in the following files:
* `MRMapper/camera/camera_constants.py`
* `MRMapper/core/ros_node/mr-mapper/src/constants.py`

### Parameter Tuning
To view and modify all parameters for the sense-making process, navigate to `MRMapper/core/ros_node/mr-mapper/src/constants.py`. You can adjust these parameters based on your specific requirements.

### RTABMap Parameters
You can fine-tune RTABMap's parameters in the launch file `MRMapper/core/ros_node/mr-mapper/launch/launch_rtabmap.xml`. For reference, you can find all of RTAB-Map's parameters listed in this [header file](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h). The "Grid/CellSize" parameter controls the density of the point cloud.

### Ports
By default, MRMapper uses TCP ports 5000-5004. If you wish to use other ports, update them accordingly in:
* `MRMapper/camera/camera_constants.py`
* `MRMapper/core/ros_node/mr-mapper/src/constants.py`
* `MRMapper/core/run.bat`
