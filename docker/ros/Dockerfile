FROM nrt/base:latest

# setup timezone
RUN echo 'Asia/Kolkata' > /etc/timezone
RUN ln -snf /usr/share/zoneinfo/Asia/Kolkata /etc/localtime
RUN add-apt-repository universe
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*


# setup sources.list
RUN echo "deb http://snapshots.ros.org/foxy/final/ubuntu focal main" > /etc/apt/sources.list.d/ros2-snapshots.list

# setup keys
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
# RUN apt-get update && apt-get install curl -y && \
# 	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg && \
# 	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
# 	curl http://repo.ros2.org/repos.key | sudo apt-key add -
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO foxy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-core=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

#RUN wget https://raw.github.com/ros/foxy/master/rosdep/sources.list.d/20-default.list && \
#	cp 20-default.list /etc/ros/rosdep/sources.list.d/

# bootstrap rosdep
RUN c_rehash /etc/ssl/certs
RUN rosdep init && \
  rosdep update --rosdistro foxy

# setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update
RUN colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml
RUN colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* ros-foxy-image-pipeline ros-foxy-ros-testing gdb \
	ros-foxy-vision-opencv libboost-serialization-dev && rm -rf /var/lib/apt/lists/*

## setup entrypoint
COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

RUN python3 -m pip install msgpack-rpc-python numpy
RUN python3 -m pip install airsim

WORKDIR /
RUN cd opencv && \
    git checkout 4.x && mkdir -p build-4 && \
    cd ..

RUN cd opencv_contrib && \
    git checkout 4.x && \
    cd ..

WORKDIR /opencv/build-4
# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") -D PYTHON3_EXECUTABLE=$(which python3) -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) -D WITH_GTK=ON -D WITH_FFMPEG=ON -D BUILD_EXAMPLES=ON -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D INSTALL_PYTHON_EXAMPLES=ON -D HAVE_opencv_python3=ON .. && \
#     make -j4 && \
#     make install
RUN make install

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
