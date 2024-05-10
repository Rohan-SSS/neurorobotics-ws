git config --global --add safe.directory /ws/

echo "============ CLONING ORBSlam3 repo ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
if [[ ! -d "ORBSlam3" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:UZ-SLAMLab/ORB_SLAM3.git
else
	echo "ORBSlam3 already present"
fi
cd ORBSlam3
echo ""
git config --global --add safe.directory /ws/ORBSlam3
cd ..

echo ""

echo "============ CLONING NRT ROS repo ============="
echo $PWD
if [[ ! -d "ros_ws" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:shandilya1998/neurorobotics-toolkit.git ros_ws
else
	echo "ROS Workspace already present"
fi

echo ""

echo "=============== CLONING MESSAGE FILTER REPO ================="
echo $PWD
#mkdir -p ros_ws/src
cd ros_ws/src/
if [[ ! -d "message_filters" ]]
then
	git clone git@github.com:shandilya1998/message_filters.git -b foxy
else
	echo "Message filter already present"
fi
cd ../../

echo "=============== CLONING CV BRIDGE REPO ================="
echo $PWD
#mkdir -p ros_ws/src
cd ros_ws/src/
if [[ ! -d "vision_opencv" ]]
then
	git clone https://github.com/ros-perception/vision_opencv.git -b foxy
else
	echo "Vision OpenCV already present"
fi
cd ../../

echo ""

echo "Done Setup"
