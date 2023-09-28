#!/bin/bash

cd /skynet_ws

./Dockerfile.sh

echo ""

echo "============ CLONING ORBSlam2 ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/ORBSlam2.git
cd ORBSlam2
echo ""
git checkout main_v1
git branch
cd ..

export CMAKE_PREFIX_PATH=/root/skynet_ws/Pangolin/build:$CMAKE_PREFIX_PATH

echo "" 

echo "============ CLONING AKAZESlam ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/AKAZESlam.git

echo ""

echo "============ CLONING SkyNet ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/SkyNet.git

echo ""

echo "============ BUILDING AKAZESlam ============="
cd ./AKAZESlam
echo $PWD
echo "Logging progress to .../AKAZESlam/build.log"
./build.sh &> build.log 
tail -n 5 build.log
cd ..

echo ""

echo "============ BUILDING ORBSlam2 ============="
cd ./ORBSlam2
echo $PWD
echo "Logging progress to .../ORBSlam2/build.log"
 ./build.sh &> build.log 
tail -n 5 build.log
cd ..

echo ""

echo "============ BUILDING SkyNet ============="
cd ./SkyNet
mkdir build && cd build
echo $PWD

echo "Running cmake. Progress logged in cmake.log file ..."
cmake .. &> cmake.log

echo ""
echo "cmake output (file cmake.log)"
echo "------------------------------------"
tail -n 5 cmake.log

echo ""
echo "Running make. Progress logged in make.log file ..."
make -j6 &> make.log

echo ""
echo "make output (file make.log)"
echo "------------------------------------"
tail -n 5 make.log
cd ../..
