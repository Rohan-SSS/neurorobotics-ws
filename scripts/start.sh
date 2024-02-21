#!/bin/bash

name="$2"
image="$3"
echo $name
echo $image
docker run --rm -d -it -v $1:/ws/ \
			-v ~/.ssh:/root/.ssh \
			-v /dev:/dev \
    		--device-cgroup-rule "c 81:* rmw" \
    		--device-cgroup-rule "c 189:* rmw" \
            -e DISPLAY=$DISPLAY \
            -v XAUTHORITY=$XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            --env="DISPLAY" --net=host \
            --privileged --name $name $image bash
