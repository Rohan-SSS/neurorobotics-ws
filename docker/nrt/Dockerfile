FROM shandilya1998/nrt:$TARGETARCH 

RUN git config --global advice.detachedHead false && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.8

# Build Pangolin
WORKDIR /deps/Pangolin/build
RUN cmake .. && \
    make -j6
