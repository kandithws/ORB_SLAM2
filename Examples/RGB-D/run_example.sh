#!/usr/bin/env sh

DATASET=${HOME}/ait_workspace/slam_datasets/rgbd_dataset_freiburg2_desk
DATASET_NUM=2
./rgbd_tum ../../Vocabulary/ORBvoc.txt TUM${DATASET_NUM}.yaml ${DATASET} associations/fr2_desk.txt