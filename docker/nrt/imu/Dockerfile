FROM shandilya1998/nrt:camera

RUN apt-get update && apt-get install -y libi2c-dev
RUN apt-get update && apt-get install ros-humble-tf2-ros ros-humble-tf2-tools

WORKDIR /deps
RUN git clone --depth=1 --branch=v11.4.3 https://github.com/machinezone/IXWebSocket.git || true && \
	cd IXWebSocket && \
	mkdir -p build && \
	cd build && \
	cmake -DBUILD_SHARED_LIBS=ON -DUSE_ZLIB=1 -DUSE_TLS=1 -DUSE_WS=1 .. && \
	make -j8 && \
	make install

WORKDIR /ws
