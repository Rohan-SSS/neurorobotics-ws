# Use a minimal Ubuntu base image
FROM ubuntu:22.04

# TODO use alpine linux (more secure and leaner): https://jfrog.com/devops-tools/article/why-use-ubuntu-as-a-docker-base-image-when-alpine-exists/
# But there are some challenges: https://unix.stackexchange.com/questions/593549/can-you-install-ubuntu-or-debian-packages-in-alpine

# Error: https://uk.mathworks.com/help/matlab/matlab_env/remove-canberra-gtk-module-and-pk-gtk-module-messages.html#
# Solution: https://askubuntu.com/questions/1175572/how-to-fix-error-failed-to-load-module-canberra-gtk-module
#           https://uk.mathworks.com/help/matlab/matlab_env/remove-canberra-gtk-module-and-pk-gtk-module-messages.html# 

# Set environment variables to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Asia/Kolkata' > /etc/timezone
RUN ln -snf /usr/share/zoneinfo/Asia/Kolkata /etc/localtime
RUN add-apt-repository universe
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt update && \
    apt install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    add-apt-repository -y ppa:deadsnakes/ppa  && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
	ca-certificates \
	ruby-dev \
	lua5.2 \
	liblua5.2-dev \
	libperl-dev \
    git\
	checkinstall \
	python3-pip \
	python3-dev \
    udev \
    sudo \
    gcc-9 \
    g++ \
    libeigen3-dev \
    cmake \
    unzip \
    pkg-config \
    libssl-dev \
	openssh-client \
	gdb \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
    rm -rf /var/lib/apt/lists/*

# Clone repositories
WORKDIR /deps

RUN git config --global advice.detachedHead false && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.8

# Build Pangolin
WORKDIR /deps/Pangolin/build
RUN cmake .. && \
    make -j6

# WORKDIR /deps/opencv/build
# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") -D PYTHON3_EXECUTABLE=$(which python3) -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) -D WITH_GTK=ON -D WITH_FFMPEG=ON -D BUILD_EXAMPLES=ON -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D INSTALL_PYTHON_EXAMPLES=ON -D HAVE_opencv_python3=ON -D WITH_GSTREAMER=ON .. && \
#     make -j2 && \
#     make install

WORKDIR /deps
RUN git clone https://github.com/vim/vim.git && cd vim && git checkout 6a950da && ./configure --with-features=huge \
            --enable-multibyte \
            --enable-rubyinterp=yes \
            --enable-python3interp=yes \
            --with-python3-config-dir=$(python3-config --configdir) \
            --enable-perlinterp=yes \
            --enable-luainterp=yes \
            --enable-gui=gtk2 \
            --enable-cscope \
            --prefix=/usr/local && \
		make VIMRUNTIMEDIR=/usr/local/share/vim/vim90 && \
		checkinstall -y

# Setup vim for development
WORKDIR /root
RUN mkdir -p /root/.vim/bundle 
RUN cd /root/.vim/bundle && git clone https://github.com/VundleVim/Vundle.vim
COPY .vimrc /root/.vimrc
RUN mkdir -p /root/.vim/colors
COPY codedark.vim /root/.vim/colors/codedark.vim 
RUN vim -c PluginInstall -c qall && ls
RUN cd /root/.vim/bundle/ && rm -rf youcompleteme && git clone https://github.com/ycm-core/YouCompleteMe youcompleteme
RUN python3 -m pip install regex
RUN cd /root/.vim/bundle/youcompleteme && git submodule update --init --recursive
RUN cd /root/.vim/bundle/youcompleteme/ && python3 ./install.py --force-sudo --clang-completer
RUN cp /root/.vim/bundle/youcompleteme/third_party/ycmd/.ycm_extra_conf.py  /root/.vim/

# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

## setup entrypoint
COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

RUN ulimit -c unlimited
RUN sysctl -w kernel.core_pattern="/tmp/%e_core.%p"
WORKDIR /ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
