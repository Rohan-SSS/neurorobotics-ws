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

echo "============ BUILDING ORBSlam3 ============="
cd /ws/ORBSlam3
echo $PWD
echo "Logging progress to .../ORBSlam3/build.log"
#./build.sh &> build.log 
./build.sh
#tail -n 5 build.log

echo ""

echo "============ BUILDING SkyNet ============="
cd /ws/SkyNet
mkdir -p build && cd build
echo $PWD

echo "Running cmake. Progress logged in cmake.log file ..."
#cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Laptop -DSLAM_MODE=orbslam3 -DINPUT_MODE=video .. &> cmake.log
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPLATFORM=Laptop -DSLAMTYPE=ORB3 -Wno-dev .. &> cmake.log

echo "" 
echo "cmake output (file cmake.log)"
echo "------------------------------------"
tail -n 5 cmake.log

echo ""
echo "Running make. Progress logged in make.log file ..."
make -j6

echo ""
echo "make output (file make.log)"
echo "------------------------------------"
tail -n 5 make.log
cd ../..
echo ""

echo "===============Building ROS2 Package=============="
cd /ws/ros_ws
rm -rf build install
cp /ws/SkyNet/build/Sensor/libSensor.so /ws/ros_ws/src/sensors/ext/libSensor.so
colcon build --symlink-install --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=1' '-DCMAKE_BUILD_TYPE=Debug' '-Wno-dev' --mixin debug
