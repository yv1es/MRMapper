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
* `groudtruth.py` This script will let you determine the position of aruco markers in the scene relative to the current camera position, usefull for groud truth evaluation. (-h for help) 

### Core 
Launch `MRMapper/docker/start.bat` to spin up a termporary container with MRMapper running. 

You can also use `MRMapper/docker/start.bat` to create a new docker container. 
If you created a container earlyer and want to reuse it, first check that the container is running. 
You can get a shell in a running docker container with this command `docker exec -it <container id> bash`.

In the container execute `roslaunch MRMapper launch.xml` to start MRMapper. 
With `vim ~/catking_ws/src/MRMapper/src/constants.py` you can tune the parameters.

### Unity 
You can find a Unity project under `MRMapper/unity`. Open Unity and launch the play view. 
The whole logic is contained under the MRMapper game object, see the attached scripts. 

After playing the scene it will connect to the core running in the container. 
It will spawn a child game object that that represents the reconstructed point cloud and it will also spawn 
other child game objects representing fitted planes and objects. 

### Summary 
1. Launch the core of MRMapper with `MRMapper/docker/start.bat` or with the command `roslaunch MRMapper launch.xml` if you already have shell in a running container.
2. Start Unity and click play to start the play view.
3. Either stream a prerecorded file from disk with `MRMapper/camera/playback.py` or connect a RealSense camera and start the stream with `MRMapper/camera/stream.py`

Now you should see the point cloud and reconstruced planes in Unity. 

## Project hierarchy 

MRMapper  
├── camera  
│   ├── Camera scripts  
│   └── Camera constants  
├── docker  
│   ├── Script to controll docker  
│   └── Dockerfile  
├── launch  
│   └── ROS launch files  
├── src  
│   ├── Python source of the ROS nodes  
│   └── constants.py Here you can tune the parameters  
└── unity  
└── MRMapper (Unity project)  

## Remarks

### Camera calibration
The `MRMapper/camera/stream.py` script will print the camera calibration of the connected RealSense camera. 
You should update the camera parameters for your RealSense. 
Update the parameters in 
* `MRMapper/camera/camera_constants.py`
* `MRMapper/src/constants.py` 

### Parameter tuning
You can view and modify all parameters for the sense making under `MRMapper/src/constants.py` 

### RTABMap parameters
You can adjust RTABMaps parameter in the launch file `MRMapper/launch/launch_rtabmap.xml`, there you can also find a link to a list of all parameters. 
The "Grid/CellSize" parameter controlls the density of the point cloud. 

### Ports 
By default MRMapper uses the TCP ports 5000-5004. If you want to use other ports update them in accordingly in 
* `MRMapper/camera/camera_constants.py`
* `MRMapper/src/constants.py`

And note that the batch scripts under `MRMapper/docker` assume the deault ports. 












