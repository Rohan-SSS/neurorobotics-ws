FROM shandilya1998/nrt:pangolin

WORKDIR /deps

RUN apt-get update && apt-get install -y libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev libcamera-tools libboost-dev openssl \
	libgnutls28-dev libgnutls28-dev qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 libcap-dev libasio-dev ninja-build && \
	python3 -m pip install meson jinja2 ninja pybind11

# Raspberry Pi specific dependencies
RUN if [ "$(uname -m)" = "aarch64" ]; then \
		apt-get update && apt-get install -y libraspberrypi-bin v4l-utils libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio; \
	elif [ "$(uname -m)" = "x86_64"]; then \
		apt-get update && apt-get install -y ros-humble-libcamera ros-humble-camera-ros libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio; \
	fi

RUN if [ "$(uname -m)" = "aarch64" ]; then \
		cd /deps && \
		git clone https://github.com/raspberrypi/libcamera.git --branch v0.3.1+rpt20240906 && \
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

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
