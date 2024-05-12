git config --global --add safe.directory /ws

# echo "============ CLONING ORBSlam3 repo ============="
# echo $PWD
# # https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# # https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
# if [[ ! -d "ORB_SLAM3" ]]
# then
# 	GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:shandilya1998/ORB_SLAM3.git 
# else
# 	echo "ORB_SLAM3 already present"
# fi
# cd ORB_SLAM3
# git checkout c++14_comp
# echo ""
# git config --global --add safe.directory /ws/ORB_SLAM3
# cd /ws

echo ""

echo "in docker-setup.sh email: $2"
echo "in docker-setup.sh username: $3"

git submodule init
git submodule update --recursive
git config --global --add safe.directory /ws/ros_ws

git config --global user.email "$2"
git config --global user.name "$3"
git config --global core.editor vim

echo ""

echo ""

echo "Done Setup"
