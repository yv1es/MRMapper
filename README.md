# MRMapper

## Installation 
### Prerequisites
A windows system. 
Following applications need to be installed on your host system: 
- [Docker](https://www.docker.com/)
- [Unity](https://unity.com)
- [Python3](https://www.python.org/) (with [virtualenv](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/))

### Docker
The core of MRMapper comes installed in a docker image. 
First clone this repository. 
On windows there is a batch script for building and running the docker image, container respectively. 
Run `MRMapper/docker/build.bat` to build the docker image called mrmapper (the image will require ~13 GB disk space and the build can take up to an hour). 

### Virtual environment
The python scripts controlling the camera run on the host system. In this step we setup a virtual environment and install all dependencies.
1. Open a powershell and navigate to the `MRMapper/camera` folder.
2. Execute `python3 -m venv env` to create a new virtual environment called "env".
3. Activate the virtual environment with `.\env\Scripts\activate`
4. Install depencendies in the virtual environment with `pip install -r requirements.txt`

Now the setup is complete! 


## Running MRMapper

There are 3 parts to MRMapper:
* Camera
* Core running in the container
* Unity

### Camera 
The system was tested with a [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) camera that was connected via a USB 3.0 cable. 

To run the following scripts make sure the virtual environment that was setup in the installation section is active in your current terminal session. 
When using powershell "(env)" should appear to the left of "PS".
If this is not the case, activate the virtual environment by executing the script located at `/MRMapper/camera/env/Scripts/activate` 

In the `MRMapper/camera` folder are several python scripts. 
* `stream.py` Start a live stream from the camera to MRMapper, use this for realtime operation. 
* `record.py` This script allows you to prerecord footage with the camera to a file on disk. (-h for help) 
* `playback.py` Use this to view recorded footage and to replay it for MRMapper, this is usefull for testing and development. (-h for help) 
* `groudtruth.py` The will let you determine the position of aruco markers in the scene relative to the current camera position, usefull for groud truth evaluation. (-h for help) 

### Core 



