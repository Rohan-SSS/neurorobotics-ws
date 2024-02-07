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

if [[ "$1" == "sitl"  ]]
then
	echo "============ BUILDING SkyNet SITL DEPENDENCIES ============="
	cd /ws/ext
	echo "============ BUILDING Memory Vendor ============="
	if [[ -d /ws/ext/foonathan_memory_vendor ]]
	then
		cd foonathan_memory_vendor
		ls
		mkdir -p build
		cd build
		ls 
		cmake ..
		cmake --build . --target install
		cd ../../
	else
		echo "========== foonathan_memory_vendor missing =========="
	fi

	echo "============ BUILDING Fast-CDR ============="
	
	if [[ -d /ws/ext/Fast-CDR ]]
	then
		cd Fast-CDR
		mkdir -p build && cd build
		cmake ..
		cmake --build . --target install
		cd ../../
	else
		echo "========== Fast-CDR MISSING =========="
	fi

	echo "============ BUILDING Fast-RTPS ============="
	if [[ -d /ws/ext/Fast-RTPS ]]
	then
		cd Fast-RTPS
		mkdir -p build && cd build
		cmake ..
		cmake --build . --target install
		cd ../../
	else
		echo "========== Fast-RTPS MISSING =========="
	fi

	echo "============ BUILDING Micro-RTPS Agent ============="
	if [[ -d /ws/ext/micrortps_agent ]]
	then
		cd micrortps_agent
		mkdir -p build && cd build
		cmake ..
		make
		cd ../../../
	else
		echo "========== micrortps_agent MISSING =========="
	fi
fi

echo "============ BUILDING SkyNet ============="
cd /ws/SkyNet
mkdir -p build && cd build
echo $PWD

echo "Running cmake. Progress logged in cmake.log file ..."
#cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Laptop -DSLAM_MODE=orbslam3 -DINPUT_MODE=video .. &> cmake.log
if [[ $1 == "sitl" ]]
then
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Drone -DSLAMTYPE=ORB2 -DAUTOPILOT=NEW .. 
else
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Laptop -DSLAMTYPE=ORB2 .. 
fi

echo "" 
echo "Running make. Progress logged in make.log file ..."
make -j6

cd ../..
echo ""
