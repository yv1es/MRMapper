FROM ubuntu:20.04

# Install tools and libs 
RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo \
    curl \ 
    lsb-release \
    vim \
    git \ 
    wget \
    dos2unix \
    software-properties-common \
    gnupg2 \
    python3-pip \
    python3-wstool \
    python3-netifaces \
    python3-opencv \
    python3-setuptools \
    build-essential 

# New version of cmake is neccessary to compile Open3d v0.17
RUN DEBIAN_FRONTEND=noninteractive apt purge -y --auto-remove cmake \
    && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
    && apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' \
    && apt update \
    && apt install -y --no-install-recommends cmake cmake-curses-gui


# Install Python packages 
RUN pip3 install pandas matplotlib opencv-contrib-python cvbridge3 mxnet gluoncv numpy==1.23.1

# Download Yolov3 weights
RUN python3 -c "from gluoncv import model_zoo, data; net = model_zoo.get_model('yolo3_darknet53_coco', pretrained=True)"


# Build and install Open3d v0.17
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
    && make -j4

RUN cd ~/Open3D/build \
    && make install \
    && make python-package \
    && pip install lib/python_package/

RUN rm -rf ~/Open3D 


# Install ROS noetic 
RUN bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN DEBIAN_FRONTEND=noninteractive apt update && apt install -y ros-noetic-ros-base

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-rosdep  \
    python3-rosinstall \ 
    python3-rosinstall-generator \      
    ros-noetic-rosbash 

RUN rosdep init && rosdep update


# Setup ROS catkin workspace
RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make


# source ROS 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


# Install RTAB-Map 
RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends ros-noetic-rtabmap
RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends ros-noetic-rtabmap-ros
RUN DEBIAN_FRONTEND=noninteractive apt remove -y ros-noetic-rtabmap-ros

# Download RTAB-Map-Ros
RUN cd ~/catkin_ws \
    && git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && cd ~/catkin_ws/src/rtabmap_ros \
    && git checkout 0.20.23-noetic

# Copy the mr-mapper ROS node into the container
COPY ros_node/mr-mapper /root/catkin_ws/src/mr-mapper

# Build workspace
RUN . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make -DCMAKE_BUILD_TYPE=RELEASE -j4 \
    && chmod +x ~/catkin_ws/src/mr-mapper/src/*.py 

# Remove the source files of mr-mapper, as they are loaded at runtime
RUN rm -rf ~/catkin_ws/src/mr-mapper
