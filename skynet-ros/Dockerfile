FROM mahesha999/opencv-realsense:0.4


WORKDIR /ws

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone
RUN ln -snf /usr/share/zoneinfo/Etc/UTC /etc/localtime
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
#RUN sudo apt update && sudo apt install curl -y && \
#	sudo curl -sSL https://raw.githubusercontent.com/ros/foxy/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# curl http://repo.ros2.org/repos.key | sudo apt-key add -
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
    build-essential \
	ca-certificates \
	vim \
    git \
	python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
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
	ros-foxy-vision-opencv && rm -rf /var/lib/apt/lists/*

# Setup vim for development
WORKDIR /root
RUN mkdir -p /root/.vim/bundle 
RUN cd /root/.vim/bundle && git clone https://github.com/VundleVim/Vundle.vim
RUN wget -O /root/.vimrc https://edward0im.github.io/files/200917/vimrc1_ycm
RUN mkdir -p /root/.vim/colors
RUN wget -O /root/.vim/colors/codedark.vim https://edward0im.github.io/files/200917/codedark.vim
RUN vim -c BundleInstall -c qall && ls
RUN cd /root/.vim/bundle/ && git clone https://github.com/ycm-core/YouCompleteMe && mv YouCompleteMe youcompleteme 
RUN cd /root/.vim/bundle/youcompleteme/ && python3 ./install.py --force-sudo --clang-completer
RUN cp  /root/.vim/bundle/youcompleteme/third_party/ycmd/.ycm_extra_conf.py  /root/.vim/

RUN python3 -m pip install msgpack-rpc-python
RUN python3 -m pip install airsim

## setup entrypoint
COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh
WORKDIR /ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
