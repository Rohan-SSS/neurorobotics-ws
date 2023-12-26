
echo "============ CLONING ORBSlam3 repo ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/ORBSlam3.git
cd ORBSlam3
echo ""
git fetch origin using_opencv44
git checkout using_opencv44
git branch
cd ..

echo ""

echo "============ CLONING ORBSlam2 repo ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/ORBSlam2.git
cd ORBSlam2
echo ""
git checkout main_v1
git branch
cd ..

echo ""


echo "============ CLONING AKAZESlam repo ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/AKAZESlam.git

echo ""


echo "============ CLONING VisionTools repo ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/VisionTools.git

echo ""

echo "============ CLONING SkyNet repo ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/SkyNet.git

echo ""

echo "=============== CLONING CV BRIDGE REPO ================="
echo $PWD
mkdir -p ros_ws/src
cd ros_ws/src/ && git clone https://github.com/ros-perception/vision_opencv.git -b foxy


echo "============ PULLING SkyNet Docker Image ============="
echo $PWD
#docker pull mahesha999/opencv-realsense:0.4

echo ""



#echo "============ CREATING SkyNet Docker Container ============="
#echo $PWD

#docker run -d -v ${PWD}/ORBSlam2:/ws/ORBSlam2 \
#            -v ${PWD}/AKAZESlam:/ws/AKAZESlam \
#            -v ${PWD}/SkyNet:/ws/SkyNet \
#            -v ${PWD}/scripts/build.sh:/ws/build.sh \
#			-v ${PWD}/scripts/docker-setup.sh:/ws/docker-setup.sh \
#            -e DISPLAY=$DISPLAY \
#            -v XAUTHORITY=$XAUTHORITY \
#            -v /tmp/.X11-unix:/tmp/.X11-unix \
#            --env="DISPLAY" --net=host \
#            --privileged --name skynet_container skynet/dev

#echo "Container started"
#echo ""



#docker exec skynet_container /bin/bash -c "/ws/build.sh"
/ws/build.sh



