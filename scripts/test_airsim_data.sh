#!/bin/bash

rm -rf SkyNet/data/data_1
mv ros_ws/data SkyNet/data/data_1
./djinn start skynet
