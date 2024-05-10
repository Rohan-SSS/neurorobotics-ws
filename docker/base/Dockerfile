# Use a minimal Ubuntu base image
FROM ubuntu:20.04

# TODO use alpine linux (more secure and leaner): https://jfrog.com/devops-tools/article/why-use-ubuntu-as-a-docker-base-image-when-alpine-exists/
# But there are some challenges: https://unix.stackexchange.com/questions/593549/can-you-install-ubuntu-or-debian-packages-in-alpine

# Error: https://uk.mathworks.com/help/matlab/matlab_env/remove-canberra-gtk-module-and-pk-gtk-module-messages.html#
# Solution: https://askubuntu.com/questions/1175572/how-to-fix-error-failed-to-load-module-canberra-gtk-module
#           https://uk.mathworks.com/help/matlab/matlab_env/remove-canberra-gtk-module-and-pk-gtk-module-messages.html# 

# Set environment variables to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
add-apt-repository -y "deb http://security.ubuntu.com/ubuntu xenial-security main"; \
fi

# Remove optional packages in production: sudo, usbutils, libcanberra-gtk3-module, iputils-ping

# Install dependencies
RUN apt update && \
    apt install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    add-apt-repository -y ppa:deadsnakes/ppa  && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
	ca-certificates \
	libncurses5-dev \
	libgtk2.0-dev \
	ruby-dev \
	lua5.2 \
	liblua5.2-dev \
	libperl-dev \
    git\
	checkinstall \
	python3-pip \
	python3-dev \
    sudo \
    usbutils \
    libcanberra-gtk3-module \
    udev \
    gcc-9 \
    g++ \
    libeigen3-dev \
    cmake \
    unzip \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
	ffmpeg \
	x264 \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    libssl-dev \
    libusb-1.0-0-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
    libyaml-cpp-dev \
    geographiclib-tools \
    libgeographic-dev \
    libspdlog-dev \
	libasio-dev \
	libtinyxml2-dev \
	libsuitesparse-dev \
	libopenexr24:amd64 \
	openssh-client \
	gdb \
	dirmngr \
	libc++1 \
	libc++abi1 \
	libicu-dev \
	libbz2-dev \
	libboost-all-dev \
	libboost-serialization-dev \
    libgtest-dev && \
    rm -rf /var/lib/apt/lists/*

RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
apt install --fix-missing -y libjasper-dev; \
fi

# Clone repositories
WORKDIR /

RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git config http.postBuffer 524288000 && \
    git checkout 4.x && \
    cd ..

RUN git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 4.x && \
    cd ..

RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    git reset --hard 61cf21520b4bb29c2c1074f7bad74a6cfcd93ca3 && \
    cd ..

RUN git config --global advice.detachedHead false && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.8

# Build librealsense 
WORKDIR /librealsense/
RUN chmod +x ./scripts/setup_udev_rules.sh && \
    ./scripts/setup_udev_rules.sh

WORKDIR /librealsense/build
RUN cmake ../ -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true && \
    make -j6 && \
    make install

# Build Pangolin
WORKDIR /Pangolin/build
RUN cmake .. && \
    make -j6

WORKDIR /

RUN cd opencv && \
    git checkout c3bb57a && mkdir build && \
    cd ..

RUN cd opencv_contrib && \
    git checkout  5fae408 && \
    cd ..

WORKDIR /opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") -D PYTHON3_EXECUTABLE=$(which python3) -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) -D WITH_GTK=ON -D WITH_FFMPEG=ON -D BUILD_EXAMPLES=ON -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D INSTALL_PYTHON_EXAMPLES=ON -D HAVE_opencv_python3=ON -D WITH_GSTREAMER=ON .. && \
    make -j4 && \
    make install

WORKDIR /ws
RUN ulimit -c unlimited
RUN sysctl -w kernel.core_pattern="/tmp/%e_core.%p"

WORKDIR /
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

WORKDIR /ws

ENTRYPOINT ["tail", "-f", "/dev/null"]
