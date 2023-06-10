# MRMapper

## Installation 
### Prerequisites
A windows system with nvidia gpu. 
Following applications need to be installed on your system: 
- Docker
- Unity
- Python with
    - pyrealsense2
    - numpy
    - opencv

### Docker
The core of MRMapper comes installed in a docker image. 
First clone this repository. 
On windows there is a batch script for building and running the docker image, container respectively. 
Run `MRMapper/docker/build.bat` to build the docker image called mrmapper. 
(Please note that for GPU support OpenCV and Open3D are build from source, this can take more than an hour to build and may take up to 30GB of disk space. 
Reducing the required disk space and build time is definitely on the todo list.)

Once the image is build you have everything setup. 

## Running 

Start Unity and open the project MRMapper wich you can find under MRMapper/unity. 
Start the unity scene, now unity is waiting for the MRMapper container. 

Connect a Intel RealSense camera to your system and execute `python MRMapper/host/stream_camera.py` 

Exectue the container with `MRMapper/docker/run.bat`. 
In the container shell execute `roslaunch MRMapper launch.xml` this shoud bring the whole system up. 



