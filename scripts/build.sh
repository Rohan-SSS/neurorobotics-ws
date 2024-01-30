#!/bin/bash

export CMAKE_PREFIX_PATH=/Pangolin/build:$CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=/opencv:$CMAKE_PREFIX_PATH

echo "build mode: $1"

echo "============ BUILDING AKAZESlam ============="
cd /ws/AKAZESlam
echo $PWD
echo "Logging progress to .../AKAZESlam/build.log"
#./build.sh &> build.log 
#tail -n 5 build.log
./build.sh

echo ""

echo "============ BUILDING ORBSlam2 ============="
cd /ws/ORBSlam2
echo $PWD
echo "Logging progress to .../ORBSlam2/build.log"
#./build.sh &> build.log 
#tail -n 5 build.log
./build.sh

echo ""

echo "============ BUILDING ORBSlam3 ============="
cd /ws/ORBSlam3
echo $PWD
echo "Logging progress to .../ORBSlam3/build.log"
#./build.sh &> build.log 
./build.sh
#tail -n 5 build.log

echo ""

if [[ "$1" == "airsim"  ]]
then
	echo "============ BUILDING SkyNet SITL DEPENDENCIES ============="
	cd /ws/ext
	echo "============ BUILDING Memory Vendor ============="
	cd foonathan_memory_vendor
	ls
	rm -rf build
	mkdir build
	cd build
	ls 
	cmake ..
	cmake --build . --target install
	cd ../../

	echo "============ BUILDING Fast-CDR ============="
	
	cd Fast-CDR
	rm -rf build && mkdir build && cd build
	cmake ..
	cmake --build . --target install
	cd ../../

	echo "============ BUILDING Fast-RTPS ============="
	cd Fast-RTPS
	rm -rf build && mkdir -p build && cd build
	cmake ..
	cmake --build . --target install
	cd ../../

	echo "============ BUILDING Micro-RTPS Agent ============="
	cd micrortps_agent
	rm -rf build && mkdir build && cd build
	cmake ..
	make
	cd ../../../

fi

echo "============ BUILDING SkyNet ============="
cd /ws/SkyNet
mkdir -p build && cd build
echo $PWD

echo "Running cmake. Progress logged in cmake.log file ..."
#cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Laptop -DSLAM_MODE=orbslam3 -DINPUT_MODE=video .. &> cmake.log
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Laptop -DSLAMTYPE=ORB2 .. &> cmake.log

echo "" 
echo "cmake output (file cmake.log)"
echo "------------------------------------"
tail -n 5 cmake.log

echo ""
echo "Running make. Progress logged in make.log file ..."
make -j6

cd ../..
echo ""
