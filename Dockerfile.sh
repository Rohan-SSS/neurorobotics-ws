echo ""
echo "============ ADDING apt repos ============="
sudo apt update && \
    sudo apt install -y --no-install-recommends software-properties-common && \
    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    # should this be added after installing libjasper-dev ?
    sudo add-apt-repository -y "deb http://security.ubuntu.com/ubuntu xenial-security main" && \

    # should this be added after installing gfortran ?
    sudo add-apt-repository -y ppa:deadsnakes/ppa 
    
echo ""
echo "============ INSTALLING libs ============="
# Install dependencies
# sudo apt-get update -y && \
#   sudo apt-get install --fix-missing -y --no-install-recommends \
sudo apt install --fix-missing -y --no-install-recommends \
	gcc-9 \
	g++ \
	libeigen3-dev \
	build-essential \
	cmake \
 	unzip \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libjasper-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    python3.6-dev \
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
    libgtest-dev \

echo ""
echo "============ CLONING OpenCV ============="
git clone https://github.com/opencv/opencv.git
cd ./opencv
echo ""
echo $PWD
echo ""
git checkout 3.4
git branch

cd ..
echo ""

echo "============ CLONING OpenCV Contrib ============="
git clone https://github.com/opencv/opencv_contrib.git
cd ./opencv_contrib/
echo ""
echo $PWD
echo ""
git checkout 3.4
git branch

cd ..
echo ""

echo "============ BUILDING OpenCV ============="
cd ./opencv
echo $PWD
mkdir build && cd build

echo ""
echo "Running cmake. Progress logged in cmake.log file ..."
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") -D PYTHON3_EXECUTABLE=$(which python3) -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) -D WITH_GTK=ON -D WITH_FFMPEG=ON -D BUILD_EXAMPLES=ON -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D INSTALL_PYTHON_EXAMPLES=ON -D HAVE_opencv_python3=ON .. &> cmake.log

echo ""
echo "cmake output (file cmake.log)"
echo "------------------------------------"
tail -n 3 cmake.log

echo ""
echo "Running make. Progress logged in make.log file ..."
make -j4 &> make.log

echo ""
echo "make output (file make.log)"
echo "------------------------------------"
tail -n 3 make.log

echo ""
echo "Running make install. Progress logged in make-install.log file ..."
make install &> make-install.log

echo ""
echo "make install output (file make-install.log)"
echo "------------------------------------"
tail -n 3 make-install.log

cd ../..
echo ""

echo "============ CLONING librealsense ============="
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

echo ""
echo $PWD

echo ""
git reset --hard 61cf21520b4bb29c2c1074f7bad74a6cfcd93ca3
git branch

cd ..
echo ""

echo "============ BUILDING librealsense ============="
cd librealsense
echo $PWD
./scripts/setup_udev_rules.sh
mkdir build && cd build

echo ""
echo "Running cmake. Progress logged in cmake.log file ..."
cmake ../ -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true &> cmake.log

echo ""
echo "cmake output (file cmake.log)"
echo "------------------------------------"
tail -n 3 cmake.log

echo ""
echo "Running make. Progress logged in make.log file ..."
make -j6 &> make.log

echo ""
echo "make output (file make.log)"
echo "------------------------------------"
tail -n 3 make.log

echo ""
echo "Running make install. Progress logged in make-install.log file ..."
make install &> make-install.log

echo ""
echo "make install output (file make-install.log)"
echo "------------------------------------"
tail -n 3 make-install.log

cd ../..
echo ""

echo "============ CLONING Pangolin ============="
echo $PWD
git config --global advice.detachedHead false
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.8

echo ""

echo "============ BUILDING Pangolin ============="
cd Pangolin/
mkdir build && cd build

echo "Running cmake. Progress logged in cmake.log file ..."
cmake .. &> cmake.log

echo ""
echo "cmake output (file cmake.log)"
echo "------------------------------------"
tail -n 3 cmake.log

echo ""
echo "Running make. Progress logged in make.log file ..."
make -j6 &> make.log

echo ""
echo "make output (file make.log)"
echo "------------------------------------"
tail -n 3 make.log

# make install #TODO ?

cd ../..
echo ""

