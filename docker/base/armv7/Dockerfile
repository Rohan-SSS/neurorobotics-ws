FROM shandilya1998/nrt:base

# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source
RUN colcon \
    build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

# setup bashrc
RUN cp /etc/skel/.bashrc ~/

ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# clone source
ARG ROS2_BRANCH=$ROS_DISTRO
ARG ROS2_REPO=https://github.com/ros2/ros2.git
RUN git clone $ROS2_REPO -b $ROS2_BRANCH \
    && vcs import src < ros2/ros2.repos

# install dependencies
RUN apt-get update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys " \
        fastcdr \
        rti-connext-dds-6.0.1 \
        urdfdom_headers" \
    && rm -rf /var/lib/apt/lists/*

# build source
RUN colcon \
    build \
    --symlink-install \
    --mixin build-testing-on release \
    --cmake-args --no-warn-unused-cli

# test build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE
RUN if [ ! -z "$RUN_TESTS" ]; then \
        colcon test; \
        if [ ! -z "$FAIL_ON_TEST_FAILURE" ]; then \
            colcon test-result; \
        else \
            colcon test-result || true; \
        fi \
    fi
WORKDIR /ws
