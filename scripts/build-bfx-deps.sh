#!/bin/bash

echo "============ BUILDING BFX SITL DEPENDENCIES ============="
if [[ -d /ws/ext/bfx ]]
then
	cd /ws/ext/bfx


	echo "============ BUILDING Memory Vendor ============="
	if [[ -d /ws/ext/bfx/foonathan_memory_vendor ]]
	then
		cd foonathan_memory_vendor
		mkdir -p build
		cd build
		cmake ..
		cmake --build . --target install
		cd ../../
	else
		echo "========== foonathan_memory_vendor missing =========="
	fi


	echo "============ BUILDING Fast-RTPS-Gen ================== "
	if [[ -d /ws/ext/bfx/Fast-RTPS-Gen ]]
	then
		cd Fast-RTPS-Gen
		gradle assemble && sudo env "PATH=$PATH" gradle install
		cd ../
	else
		echo "========== Fast-RTPS-Gen MISSING =========="
	fi
			
	echo "============ BUILDING Fast-CDR ============="	
	if [[ -d /ws/ext/bfx/Fast-CDR ]]
	then
		cd Fast-CDR
		mkdir -p build && cd build
		cmake ..
		cmake --build . --target install
		cd ../../
	else
		echo "========== Fast-CDR MISSING =========="
	fi
		
	echo "============ BUILDING Fast-DDS ================== "
	if [[ -d /ws/ext/bfx/FastDDS-2.0.2 ]]
	then
		cd FastDDS-2.0.2
		mkdir -p build
		cd build
		cmake ..
		cmake --build . --target install
		cd ../../
	else
		echo "========== FastDDS-2.0.2 MISSING =========="
	fi
	

	cd ../../../
else
	echo "========== BFX DEPENDENCIES NOT PLACED AT ext/ =========="
fi
