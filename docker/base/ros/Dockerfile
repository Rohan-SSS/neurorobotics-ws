FROM shandilya1998/nrt:base

RUN apt-get update && apt-get install -y --no-install-recommends curl && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-ros-core=0.10.0-1* && \
	rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && \
	apt-get install -y --no-install-recommends ros-humble-ros-base=0.10.0-1* && \
	apt-get install -y --no-install-recommends ros-humble-desktop=0.10.0-1* &&  \
	apt-get install -y --no-install-recommends ros-humble-desktop-full=0.10.0-1* && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
	python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool && \
	rm -rf /var/lib/apt/lists/* && \
	c_rehash /etc/ssl/certs && \
	rosdep init && rosdep update --rosdistro humble

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
	colcon mixin update && \
	colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
	colcon metadata update


RUN apt-get update && apt-get install -y libi2c-dev ros-humble-tf2-ros ros-humble-tf2-tools

RUN ulimit -c unlimited
RUN sysctl -w kernel.core_pattern="/tmp/%e_core.%p"
WORKDIR /ws

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
