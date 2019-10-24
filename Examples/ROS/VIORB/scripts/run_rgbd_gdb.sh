#!/usr/bin/env sh

VOCAB_FILE=${HOME}/ait_workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt
CONFIG_FILE=$(rospack find VIORB)/config/rgbd_vi/ETH3D.yaml
SENSOR_TYPE=RGBD

rosrun --prefix 'gdb -ex=r --args' VIORB stereo ${VOCAB_FILE} ${CONFIG_FILE} _sensor_type:=${SENSOR_TYPE}
