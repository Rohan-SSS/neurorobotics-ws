#!/bin/bash

docker exec -it bfx_container /bin/bash -l -c "export BFX=\"/ws/ext/bfx/BFX\" && \${BFX}/Tools/sitl_run.sh \${BFX}/build/px4_sitl_rtps/bin/px4 none none iris none \${BFX} \${BFX}/build/px4_sitl_rtps"
