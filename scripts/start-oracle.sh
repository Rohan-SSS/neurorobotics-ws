#!/bin/bash
cd $WS_PATH/skynet-oracle
flask --app "oracle:create_app('$WS_PATH')" run --host=0.0.0.0 --port 8080 --debug
cd -
