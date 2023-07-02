
FROM ros:noetic-ros-core

# upgrade python version to python3.10
RUN apt update \
    && apt install software-properties-common -y \
    && add-apt-repository ppa:deadsnakes/ppa -y \
    && apt update \
    && apt install python3.10 -y \
    && sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

RUN sudo apt install curl -y \
    && curl https://bootstrap.pypa.io./get-pip.py | python3 \
    && bash -c "echo 'export PATH="/usr/local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc"



# install basic tools
RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo \
    curl \ 
    lsb-release \
    vim \
    git \ 
    wget \
    software-properties-common \
    python3 \
    python3-dev \
    python3-pip \
    python3-testresources \
    python3-tk



# downgrade numpy (for gluoncv)
RUN pip uninstall -y numpy && pip install numpy==1.23.1
 

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-rosdep  \
    python3-rosinstall \ 
    python3-rosinstall-generator \  
    python3-wstool \
    build-essential \
    ros-noetic-rosbash

RUN sudo rosdep init && rosdep update

# setup catkin workspace 
RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make

# add source commands to bashrc 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc





# opencv with cuda
RUN sudo apt-get update \
    && sudo apt-get install -y python3-opencv

RUN pip3 install numpy pandas matplotlib opencv-contrib-python cvbridge3 mxnet gluoncv open3d





# Install RTABmap 
RUN sudo apt install -y ros-noetic-rtabmap

RUN cd ~/catkin_ws \
    && git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && cd ~/catkin_ws/src/rtabmap_ros && git checkout 0.20.23-noetic


RUN cd ~/catkin_ws/src \
    && git clone https://github.com/yv1es/MRMapper.git \
    && cd ~/catkin_ws/src/MRMapper \
    && git checkout main

RUN sudo apt install -y ros-noetic-rtabmap-ros
RUN sudo apt remove -y ros-noetic-rtabmap-ros

# install ROS packages
RUN . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make -DCMAKE_BUILD_TYPE=RELEASE -j10 \
    && chmod +x ~/catkin_ws/src/MRMapper/src/*.py

RUN python3 -c "from gluoncv import model_zoo, data; net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)"
