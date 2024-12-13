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
RUN echo 'Asia/Kolkata' > /etc/timezone && \
	ln -snf /usr/share/zoneinfo/Asia/Kolkata /etc/localtime && \
	apt-get update && \
	apt-get install -q -y --no-install-recommends tzdata gpg-agent && \
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
	python3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
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
	bash-completion \
    python3-argcomplete \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout && \
    rm -rf /var/lib/apt/lists/*

# Clone repositories
WORKDIR /deps

# WORKDIR /deps/opencv/build
# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") -D PYTHON3_EXECUTABLE=$(which python3) -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) -D WITH_GTK=ON -D WITH_FFMPEG=ON -D BUILD_EXAMPLES=ON -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D INSTALL_PYTHON_EXAMPLES=ON -D HAVE_opencv_python3=ON -D WITH_GSTREAMER=ON .. && \
#     make -j2 && \
#     make install

WORKDIR /deps
RUN git clone https://github.com/vim/vim.git && cd vim && git checkout  6cc3027 && ./configure --with-features=huge \
            --enable-multibyte \
            --enable-rubyinterp=yes \
            --enable-python3interp=yes \
            --with-python3-config-dir=$(python3-config --configdir) \
            --enable-perlinterp=yes \
            --enable-luainterp=yes \
            --enable-gui=gtk2 \
            --enable-cscope \
            --prefix=/usr/local && \
		make VIMRUNTIMEDIR=/usr/local/share/vim/vim91 && \
		checkinstall -y

# Setup vim for development
WORKDIR /root
RUN mkdir -p /root/.vim/bundle && \
	cd /root/.vim/bundle && \
	git clone https://github.com/VundleVim/Vundle.vim && \
	mkdir -p /root/.vim/colors
COPY ./.vimrc /root/.vimrc
COPY ./codedark.vim /root/.vim/colors/codedark.vim 
RUN vim -c PluginInstall -c qall && ls && \
	cd /root/.vim/bundle/ && \
	rm -rf youcompleteme && \
	git clone https://github.com/ycm-core/YouCompleteMe youcompleteme && \
	python3 -m pip install regex && \
	cd /root/.vim/bundle/youcompleteme && \
	git submodule update --init --recursive && \
	python3 ./install.py --force-sudo --clang-completer && \
	cp /root/.vim/bundle/youcompleteme/third_party/ycmd/.ycm_extra_conf.py  /root/.vim/
