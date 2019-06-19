#!/usr/bin/env sh

DATASET=/media/backup/KITTI/sequences/00
CONFIG_FILE=00-02
gdb -ex=r --args ./mono_kitti ../../Vocabulary/ORBvoc.txt KITTI${CONFIG_FILE}.yaml ${DATASET}
