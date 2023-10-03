echo "============ BUILDING SkyNet Docker Image ============="
echo $PWD
# docker buildx build . --no-cache --progress plain --platform linux/arm64/v8,linux/amd64 -t mahesha999/opencv-realsense:0.4 &> docker-build.log
# docker buildx build . --no-cache --progress plain --platform linux/arm64/v8 -t mahesha999/opencv-realsense:0.4 &> docker-build.log
docker buildx build . --progress plain --platform linux/arm64/v8,linux/amd64 -t mahesha999/opencv-realsense:0.4 --push &> docker-build.log