#!/bin/bash

export CMAKE_PREFIX_PATH=/Pangolin/build:$CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=/opencv:$CMAKE_PREFIX_PATH

echo "build mode: $1"


echo "============ BUILDING ORBSlam3 ============="
cd /ws/ORBSlam3
echo $PWD
echo "Logging progress to .../ORBSlam3/build.log"
#./build.sh &> build.log 
./build.sh
#tail -n 5 build.log
cd /ws

echo ""

echo "" 
echo "Running make. Progress logged in make.log file ..."
make -j6

cd ../..
echo ""
