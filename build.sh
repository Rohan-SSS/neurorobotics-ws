export CMAKE_PREFIX_PATH=/ws/Pangolin/build:$CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=/ws/opencv:$CMAKE_PREFIX_PATH


echo "============ BUILDING AKAZESlam ============="
cd /ws/AKAZESlam
echo $PWD
echo "Logging progress to .../AKAZESlam/build.log"
./build.sh &> build.log 
tail -n 5 build.log

echo ""

echo "============ BUILDING ORBSlam2 ============="
cd /ws/ORBSlam2
echo $PWD
echo "Logging progress to .../ORBSlam2/build.log"
./build.sh &> build.log 
tail -n 5 build.log

echo ""

echo "============ BUILDING SkyNet ============="
cd /ws/SkyNet
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
