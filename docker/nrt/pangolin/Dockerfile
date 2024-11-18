FROM shandilya1998/nrt:ros 

WORKDIR /deps

RUN git config --global advice.detachedHead false && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
	cd Pangolin && git checkout tags/v0.8 -b v0.8

# Build Pangolin
WORKDIR /deps/Pangolin/build
RUN cmake .. && \
   	cmake --build . -j4 && \
	cmake --install .

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

WORKDIR /ws
