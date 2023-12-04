#!/bin/bash

if [[ "$1" == "skynet" ]]
then
	cd SkyNet/build
	cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=Laptop -DSLAM_MODE=orbslam2 -DINPUT_MODE=video ..
i
