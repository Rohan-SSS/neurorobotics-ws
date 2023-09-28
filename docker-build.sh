echo "============ BUILDING SkyNet Docker Image ============="
echo $PWD
docker build . --no-cache --progress plain -t mahesha999/opencv-realsense:give-desired-tag &> docker-build.log