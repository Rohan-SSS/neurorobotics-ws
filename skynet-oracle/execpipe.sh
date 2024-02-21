#!/bin/bash
# Refer to the following link for more information about why named pipes have been used here:
# https://stackoverflow.com/questions/32163955/how-to-run-shell-script-on-host-from-docker-container
while true; do eval "$(cat pipe/djinn_pipe)"; done
