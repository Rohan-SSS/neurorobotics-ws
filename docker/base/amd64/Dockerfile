FROM shandilya1998/nrt:base

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-ros-core=0.10.0-1* && \
	rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && \
	apt-get install -y --no-install-recommends ros-humble-ros-base=0.10.0-1* && \
	apt-get install -y --no-install-recommends ros-humble-desktop=0.10.0-1* &&  \
	apt-get install -y --no-install-recommends ros-humble-desktop-full=0.10.0-1* && \
    rm -rf /var/lib/apt/lists/*

RUN ulimit -c unlimited
RUN sysctl -w kernel.core_pattern="/tmp/%e_core.%p"
WORKDIR /ws
