#!/bin/bash
export FLASK_APP="$WS_PATH"/skynet-oracle/
flask --app "oracle:create_app('$HOME/skynet-ws')" run --host=0.0.0.0 --debug
