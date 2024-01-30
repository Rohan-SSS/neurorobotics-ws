
FROM mahesha999/opencv-realsense:0.4

RUN python3 -m pip install msgpack-rpc-python numpy
RUN python3 -m pip install airsim

RUN apt-get update && apt-get install -y libasio-dev libtinyxml2-dev libsuitesparse-dev
WORKDIR /ws

ENTRYPOINT ["tail", "-f", "/dev/null"]
