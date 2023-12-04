echo "============ BUILDING SkyNet Docker Image ============="
echo $PWD
# docker buildx build . --no-cache --progress plain --platform linux/arm64/v8,linux/amd64 -t mahesha999/opencv-realsense:0.4 &> docker-build.log
# docker buildx build . --no-cache --progress plain --platform linux/arm64/v8 -t mahesha999/opencv-realsense:0.4 &> docker-build.log
echo $1
echo $2
echo $3
#docker buildx build $3 --progress plain --platform $1 -t $2 --push &> docker-build.log
docker build -t $2 $3 --add-host "raw.githubusercontent.com:151.101.84.133"
