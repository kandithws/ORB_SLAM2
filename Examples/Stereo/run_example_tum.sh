#!/usr/bin/env bash

# DATASET=/media/backup/thesis_dataset/dinning_room/ds1
# CONFIG=thesis_datasets/dining_room_tum.yaml

# Chair2 Data set
#DATASET=/media/backup/thesis_dataset/chair2/ds3
#CONFIG=thesis_datasets/chair.yaml

# TREE Dataset
DATASET=/media/backup/thesis_dataset/tree_datasets/tree3-2/1
CONFIG=./thesis_datasets/tree3-2.yaml
# DATASET=/media/backup/thesis_dataset/tree_datasets/tree3-3/1
# CONFIG=./thesis_datasets/tree.yaml
./stereo_tum ../../Vocabulary/ORBvoc.txt ${CONFIG} ${DATASET} ${DATASET}/association.txt
