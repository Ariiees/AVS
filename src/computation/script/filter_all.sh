#!/bin/bash
set -e

echo "Starting Tasks..."
for i in $(seq -w 0 10); do
  echo "Filter sequence $i"
  /home/yuxw/computation/build//computation /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/$i/velodyne_old /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/$i/velodyne
done
echo "Task Complete"

