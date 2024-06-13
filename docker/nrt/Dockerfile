FROM shandilya1998/nrt:ros 

WORKDIR /deps

RUN git config --global advice.detachedHead false && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
	cd Pangolin && git checkout tags/v0.8 -b v0.8

# Build Pangolin
WORKDIR /deps/Pangolin/build
RUN cmake .. && \
    make -j6

WORKDIR /ws
