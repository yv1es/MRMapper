FROM ubuntu:20.04

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
    gnupg2 \
    python3-pip


# Install newer version of cmake necessary for Open3d
RUN apt purge -y --auto-remove cmake \
    && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
    && apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' \
    && apt update \
    && apt install -y cmake cmake-curses-gui


# install libs
RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-wstool \
    python3-netifaces \
    python3-opencv \
    python3-setuptools \
    build-essential 



# pip install numpy==1.23.5
# pip install mxnet -f https://dist.mxnet.io/python/cpu
RUN pip3 install pandas matplotlib opencv-contrib-python cvbridge3 mxnet gluoncv numpy==1.23.1


# build and install open3d (build from source for version 0.17)
RUN cd ~ \
    && git clone https://github.com/isl-org/Open3D \
    && cd ~/Open3D \
    && git checkout v0.17.0

RUN cd ~/Open3D \
    && util/install_deps_ubuntu.sh assume-yes

RUN cd ~/Open3D \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D BUILD_EXAMPLES=OFF ..

RUN cd ~/Open3D/build \
    && make -j$(nproc) 

RUN cd ~/Open3D/build \
    && make install \
    && make python-package \
    && pip install lib/python_package/

RUN rm -rf ~/Open3D 



# install ROS noetic 
RUN bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y ros-noetic-ros-base

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-rosdep  \
    python3-rosinstall \ 
    python3-rosinstall-generator \      
    ros-noetic-rosbash 

RUN rosdep init && rosdep update


# setup catkin workspace 
RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make


# add source commands for ROS to bashrc 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


# Install RTABmap 
RUN apt install -y ros-noetic-rtabmap

RUN apt install -y ros-noetic-rtabmap-ros
RUN apt remove -y ros-noetic-rtabmap-ros

RUN cd ~/catkin_ws \
    && git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && cd ~/catkin_ws/src/rtabmap_ros && git checkout 0.20.23-noetic


# MRMapper
RUN cd ~/catkin_ws/src \
    && git clone https://github.com/yv1es/MRMapper.git \
    && cd ~/catkin_ws/src/MRMapper 
    # && git checkout main


# build ROS packages in workspace
RUN . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make -DCMAKE_BUILD_TYPE=RELEASE -j$(nproc) \
    && chmod +x ~/catkin_ws/src/MRMapper/src/*.py


# preload yoloV3 model weights
RUN python3 -c "from gluoncv import model_zoo, data; net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)"
