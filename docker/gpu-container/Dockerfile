
FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

# FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

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
    python3-testresources 

 
# Install newer version of cmake
# RUN sudo apt purge --auto-remove cmake \
#     && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
#     && sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' \
#     && sudo apt update \
#     && sudo apt install -y cmake cmake-curses-gui


# # install ROS noetic
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-noetic-desktop   

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-rosdep  \
    python3-rosinstall \ 
    python3-rosinstall-generator \  
    python3-wstool \
    python3-pip \
    build-essential \
    git

RUN sudo rosdep init && rosdep update

# setup catkin workspace 
RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make

# # add source commands to bashrc 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc



# # Qt5 Qhull boost etc.
# RUN apt update \
#     && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
#     build-essential g++ autotools-dev libicu-dev libbz2-dev libboost-all-dev \
#      qtbase5-dev  qtdeclarative5-dev qtquickcontrols2-5-dev libqt5opengl5-dev \
#      libqhull-dev \
#      libblas-dev liblapack-dev libgmp3-dev libmpfr-dev libopenblas-dev


# # Eigen
# RUN cd ~ \
#     && git clone https://gitlab.com/libeigen/eigen.git \
#     && cd ~/eigen \
#     && git checkout 3.4.0 \
#     && mkdir build \
#     && cd build \
#     && cmake -D CMAKE_BUILD_TYPE=RELEASE .. 

# RUN cd ~/eigen/build \
#     && make -j20 \
#     && make install 



# # SuitSparse
# RUN cd ~ \
#     && git clone https://github.com/DrTimothyAldenDavis/SuiteSparse.git

# RUN cd ~/SuiteSparse \
#     && git checkout stable \
#     # fix a bug where cublas is not found  Install cuda later
#     && ln -s /usr/local/cuda-11.7/targets/x86_64-linux/include/cublas_v2.h  /usr/include/cublas_v2.h  \   
#     && ln -s /usr/local/cuda-11.7/targets/x86_64-linux/include/cublas_api.h  /usr/include/cublas_api.h  \
#     && CMAKE_OPTIONS="-DENABLE_CUDA=OFF  \
#                       -DSUITESPARSE_CUDA_ARCHITECTURES=30,35,50,52,60,61,70,75,86 \
#                       -DCUDA_INCLUDE_DIRS=/usr/local/cuda-11.7/include \
#                       -DBLA_VENDOR=OpenBLAS \
#                      " make global -j20 \
#     && make install 



# # G2O
# RUN cd ~ \
#     && git clone https://github.com/RainerKuemmerle/g2o.git 

# RUN cd ~/g2o \
#     && git checkout 20201223_git \
#     && mkdir build \
#     && cd build \
#     && cmake -D G2O_FAST_MATH=ON \
#              -D BUILD_WITH_MARCH_NATIVE=ON \ 
#              -D CMAKE_BUILD_TYPE=RELEASE ..

# RUN cd ~/g2o/build \
#     && make -j20 \
#     && make install 


# # VTK 
# # RUN cd ~ \
# #     && git clone https://github.com/Kitware/VTK.git 

# # RUN cd ~/VTK \
# #     && git checkout v9.2.6 \
# #     && mkdir build \
# #     && cd build \
# #     && cmake -D VTK_USE_CUDA=ON \
# #              -D VTK_GROUP_ENABLE_Qt=YES \
# #              -D VTK_MODULE_ENABLE_VTK_GUISupportQtQuick=DONT_WANT \
# #              -D CMAKE_BUILD_TYPE=RELEASE .. 

# # RUN cd ~/VTK/build \
# #     && make -j20 \
# #     && make install 




# opencv gpu
RUN sudo apt-get update \
    && sudo apt-get install -y libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    libjpeg-dev libpng-dev libtiff-dev libatlas-base-dev gfortran libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev libvorbis-dev libopencore-amrnb-dev \
    libopencore-amrwb-dev 

RUN pip3 install numpy pandas matplotlib 

RUN sudo apt-get update \
    && sudo apt-get install -y libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils \
    && cd /usr/include/linux \
    && sudo ln -s -f ../libv4l1-videodev.h videodev.h \
    && cd ~

RUN sudo apt-get install -y libgtk-3-dev

RUN sudo apt-get install -y libtbb-dev
RUN sudo apt-get install -y libatlas-base-dev gfortran unzip

RUN sudo apt-get update \
    && sudo apt-get install -y libprotobuf-dev protobuf-compiler \
    && sudo apt-get install -y libgoogle-glog-dev libgflags-dev \
    && sudo apt-get install -y libgphoto2-dev libhdf5-dev doxygen

RUN cd ~ \
    && wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.2.zip \
    && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.2.zip \
    && unzip opencv.zip \
    && unzip opencv_contrib.zip


RUN cd ~/opencv-4.5.2 \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D WITH_TBB=ON \
        -D ENABLE_FAST_MATH=1 \
        -D CUDA_FAST_MATH=1 \
        -D WITH_CUBLAS=1 \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D BUILD_opencv_cudacodec=ON \
        -D OPENCV_DNN_CUDA=ON \
        -D CUDA_ARCH_BIN=6.0,6.1,7.0,7.5,8.6 \
        -D WITH_V4L=ON \
        -D WITH_OPENGL=OFF \
        -D WITH_GSTREAMER=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_PC_FILE_NAME=opencv.pc \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-4.5.2/modules \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D BUILD_JPEG=ON \
        -D BUILD_opencv_python3=ON \
        -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3 \
        -D INSTALL_C_EXAMPLES=OFF ..

        # -D WITH_QT=ON \
        # -D BUILD_opencv_python3=ON \
        # BUILD_opencv_rgbd=OFF
        # -D BUILD_opencv_python=ON \
        # -D WITH_PYTHON=ON \
        # -D BUILD_opencv_python3=ON \
        # -D OPENCV_PYTHON3_VERSION=ON \
        
        # -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
        # -D PYTHON_INCLUDE_DIR=/usr/include/python3.8 \
        # -D PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so \
        
        # -D INSTALL_C_EXAMPLES=OFF .. 
    
RUN cd ~/opencv-4.5.2/build \
    && make -j10 \
    && make install






# # PCL
# # RUN sudo apt install -y libusb-1.0-0-dev libflann-dev

# # RUN cd ~ \
# #     && git clone https://github.com/PointCloudLibrary/pcl.git 

# # RUN cd ~/pcl \
# #     && git checkout pcl-1.13.0 \
# #     && mkdir build \
# #     && cd build \
# #     && cmake -D BUILD_GPU=ON \
# #         -D BUILD_CUDA=ON \
# #         -D CMAKE_BUILD_TYPE=RELEASE .. 

# # RUN cd ~/pcl/build \
# #     && make -j10 \
# #     && make install 


 
RUN sudo apt install -y ros-noetic-rtabmap ros-noetic-rtabmap-ros
RUN sudo apt remove -y ros-noetic-rtabmap ros-noetic-rtabmap-ros

# # Install RTABmap 
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    libsqlite3-dev  libproj-dev libqt5svg5-dev 



# RUN sudo apt-get install -y libvtk7-dev libpcl-dev ros-noetic-pcl-ros


RUN cd ~ \
    && git clone https://github.com/introlab/rtabmap.git rtabmap 

RUN cd ~/rtabmap/build \
    && cmake .. \ 
    && make -j20 \
    && sudo make install



# RUN  cd ~/catkin_ws/src \
#     && git clone https://github.com/ros-perception/perception_pcl.git \
#     && cd ~/catkin_ws/src/perception_pcl \
#     && git checkout melodic-devel


# # rtabmap_ros
RUN pip install cvbridge3

RUN cd ~/catkin_ws \
    && git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && cd ~/catkin_ws/src/rtabmap_ros && git checkout 0.20.23-noetic

# RUN cd ~/catkin_ws/src \
#     && git clone https://github.com/ros-planning/navigation_msgs.git

# RUN sudo apt-get install -y ros-noetic-tf2-geometry-msgs ros-noetic-pcl-msgs  ros-noetic-image-geometry ros-noetic-tf2-eigen

# RUN cd ~/catkin_ws \ 
#     catkin_make -DRTABMAP_GUI=ON -DCMAKE_BUILD_TYPE=RELEASE -j20


# install realsense stream packet
RUN cd ~/catkin_ws/src \
    && git clone https://github.com/yv1es/MRMapper.git \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make -DRTABMAP_GUI=ON -DCMAKE_BUILD_TYPE=RELEASE -j10 \
    && chmod +x ~/catkin_ws/src/MRMapper/src/*.py
