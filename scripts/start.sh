#!/bin/bash

docker run --rm -d -it -v $1/ORBSlam2:/ws/ORBSlam2 \
            -v $1/AKAZESlam:/ws/AKAZESlam \
            -v $1/VisionTools:/ws/VisionTools \
            -v $1/ORBSlam3:/ws/ORBSlam3 \
            -v $1/SkyNet:/ws/SkyNet \
			-v $1/src:/ws/ros_ws/src \
            -v $1/scripts/build.sh:/ws/build.sh \
			-v $1/scripts/docker-setup.sh:/ws/docker-setup.sh \
			-v ~/.ssh:/root/.ssh \
			-v /dev:/dev \
    		--device-cgroup-rule "c 81:* rmw" \
    		--device-cgroup-rule "c 189:* rmw" \
            -e DISPLAY=$DISPLAY \
            -v XAUTHORITY=$XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            --env="DISPLAY" --net=host \
            --privileged --name skynet_container skynet/dev bash
