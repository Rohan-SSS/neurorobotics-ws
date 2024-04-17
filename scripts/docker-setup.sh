git config --global --add safe.directory /ws/

echo "============ CLONING ORBSlam3 repo ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
if [[ ! -d "ORBSlam3" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/ORBSlam3.git
else
	echo "ORBSlam3 already present"
fi
cd ORBSlam3
echo ""
git config --global --add safe.directory /ws/ORBSlam3
git fetch origin using_opencv44
# git checkout using_opencv44
git branch
cd ..

echo ""

echo "============ CLONING ORBSlam2 repo ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
if [[ ! -d "ORBSlam2" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/ORBSlam2.git
else
	echo "ORBSlam2 already present"
fi
cd ORBSlam2
git config --global --add safe.directory /ws/ORBSlam2
echo "build mode: $1"
# git checkout main_v1
# if [[ "$1" == "sitl" ]]
# then
# 	git checkout vaibhav_covariance
# else
# 	git checkout main_v1
# fi
git branch
cd ..

echo ""


echo "============ CLONING AKAZESlam repo ============="
echo $PWD
if [[ ! -d "AKAZESlam" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/AKAZESlam.git
else
	echo "AKAZESlam already present"
fi
cd AKAZESlam
git config --global --add safe.directory /ws/AKAZESlam
cd ../

echo ""


echo "============ CLONING VisionTools repo ============="
echo $PWD
if [[ ! -d "VisionTools" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/VisionTools.git
else
	echo "VisionTools already present"
fi
cd VisionTools
git config --global --add safe.directory /ws/VisionTools
cd ../

echo ""

mkdir -p ext && cd ext/
if [[ ! -d "Fast-CDR" ]]
then
	git clone https://github.com/eProsima/Fast-CDR.git
	cd Fast-CDR
	git checkout c69dff2
	cd ../
else
	echo "Fast-CDR already present"
fi
if [[ ! -d "foonathan_memory_vendor" ]]
then
	git clone https://github.com/eProsima/foonathan_memory_vendor.git
else
	echo "foonathan_memory_vendor already present"
fi
echo "=============== CLONING FAST-RTPS REPOS ================="
if [[ ! -d "Fast-RTPS" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/Fast-RTPS.git
else
	echo "Fast-RTPS already present"
fi
cd /ws

echo ""

echo "============ CLONING SkyNet repo ============="
echo $PWD
if [[ ! -d "SkyNet" ]]
then
	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/SkyNet.git
else
	echo "SkyNet already present"
fi
cd SkyNet
git config --global --add safe.directory /ws/SkyNet
# git checkout main_v1
git submodule init
git submodule update
cd ../

echo ""

if [[ "$1" == "sitl" ]]
then
	echo "Not setting up ros dependencies"
else
	echo "============ CLONING SkyNet-ROS repo ============="
	echo $PWD
	if [[ ! -d "ros_ws" ]]
	then
		GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/skynet-ros.git ros_ws
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
		git clone git@github.com:shandilya1b/message_filters.git -b foxy
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

	# echo "============ PULLING SkyNet Docker Image ============="
	# echo $PWD
	#docker pull mahesha999/opencv-realsense:0.4

	echo ""
fi	

echo "=============== CLONING KALIBR REPO ================="
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
cp Dockerfile_ros1_20_04 Dockerfile

echo "Done Setup"
