FROM introlab3it/rtabmap:focal


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Will be used to read/store databases on host
RUN mkdir -p /root/Documents/RTAB-Map


RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo \
    curl \ 
    lsb-release \
    vim \
    git \ 
    wget \
    python3-pip \
    ros-noetic-fiducial-msgs


# setup catkin workspace 
RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make

# add source command to bashrc 
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc



RUN cd ~/catkin_ws \
    && git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && cd ~/catkin_ws/src/rtabmap_ros && git checkout 0.20.23-noetic

RUN cd ~/catkin_ws/src \
    && git clone https://github.com/ros-planning/navigation_msgs.git

RUN sudo apt-get install -y ros-noetic-tf2-geometry-msgs ros-noetic-pcl-msgs  ros-noetic-image-geometry ros-noetic-tf2-eigen


RUN pip install cvbridge3
# install realsense stream packet
RUN cd ~/catkin_ws/src \
    && git clone https://github.com/yv1es/MRMapper.git \
    && cd ~/catkin_ws/src/MRMapper/src \
    && chmod +x * \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make -DRTABMAP_GUI=ON -DCMAKE_BUILD_TYPE=RELEASE -j4