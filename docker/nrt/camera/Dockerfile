FROM shandilya1998/nrt:pangoling

WORKDIR /deps

RUN apt-get update && apt-get install -y libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev libcamera-tools libboost-dev openssl \
	libgnutls28-dev libgnutls28-dev qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 libcap-dev 

RUN if [ "$(uname -m)" = "aarch64" ]; then \
		cd /deps && \
		git clone https://github.com/raspberrypi/libcamera.git && \
		cd libcamera && \
		meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled && \
		ninja -C build install;\
	fi

RUN if [ "$(uname -m)" = "aarch64" ]; then \
		cd /deps && \
		git clone https://github.com/raspberrypi/rpicam-apps.git && \
		cd rpicam-apps && \
		meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=enabled -Denable_tflite=disabled && \
		meson compile -C build && \
		meson install -C build && \		
		ldconfig && \
		git clone https://github.com/tomba/kmsxx.git && \
		cd kmsxx && \
		git submodule update --init && \
		meson build -Dpykms=enabled && \
		ninja -C build install && \
		python3 -m pip install rpi-libcamera rpi-kms picamera2; \
	fi

RUN python3 -m pip install av
RUN git clone https://github.com/damiafuentes/DJITelloPy.git && \
	cd DJITelloPy && \
	pip install -e .

WORKDIR /ws
