#!/usr/bin/env bash

# DATASET=/media/backup/thesis_dataset/dinning_room/ds1
# CONFIG=thesis_datasets/dining_room_tum.yaml

# Chair2 Data set
#DATASET=/media/backup/thesis_dataset/chair2/ds3
#CONFIG=thesis_datasets/chair.yaml

# TREE Dataset
# DATASET=/media/backup/thesis_dataset/tree_datasets/tree2/1
DATASET=/home/kandithws/tree_dataset2/tree2-2/1
CONFIG=thesis_datasets/tree.yaml
./stereo_tum ../../Vocabulary/ORBvoc.txt ${CONFIG} ${DATASET} ${DATASET}/association.txt
