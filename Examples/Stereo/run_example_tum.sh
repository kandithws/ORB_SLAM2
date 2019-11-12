#!/usr/bin/env bash

DATASET=/media/backup/thesis_dataset/dinning_room/ds1
CONFIG=thesis_datasets/dining_room_tum.yaml
./stereo_tum ../../Vocabulary/ORBvoc.txt ${CONFIG} ${DATASET} ${DATASET}/association.txt