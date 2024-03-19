#!/bin/bash

MODE=$1

if [[ "$MODE" == "automation" ]]
then
	docker exec -it sitl_container /bin/bash -l -c "nohup bash -c \"cd /ws/ext/micrortps_agent/build && ./micrortps_agent -t UDP\" > rtps_agent.log &"
else	
	docker exec -it sitl_container /bin/bash -l -c "cd /ws/ext/micrortps_agent/build && ./micrortps_agent -t UDP"
fi
