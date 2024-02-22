#!/bin/bash

docker exec -it bfx_container /bin/bash -l -c "cd /ws && nohup mavlink-routerd -e 192.168.26.78:14550 127.0.0.1:14550 >> mavlink.log &"
docker exec -it bfx_container /bin/bash -l -c "sleep 5"
docker exec -it bfx_container /bin/bash -l -c "export BFX=\"/ws/ext/bfx/BFX\" && \${BFX}/Tools/sitl_run.sh \${BFX}/build/px4_sitl_rtps/bin/px4 none none iris none \${BFX} \${BFX}/build/px4_sitl_rtps"
# BFX="${PWD}/ext/bfx/BFX"
# $BFX/Tools/sitl_run.sh $BFX/build/px4_sitl_rtps/bin/px4 none airsim iris Africa $BFX $BFX/build/px4_sitl_rtps
