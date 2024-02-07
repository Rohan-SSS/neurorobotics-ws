#!/bin/bash

docker exec -it sitl_container /bin/bash -l -c "cd /ws/ext/micrortps_agent/build && ./micrortps_agent -t UDP"
