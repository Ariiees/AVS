#!/bin/bash
set -e

if [ $# -ne 1 ]; then
  echo "Usage: $0 <data_name>"
  exit 1
fi

data_name=$1

echo "Starting tasks..."
for i in $(seq -w 0 10); do
  echo "Count sequence $i"
  /home/yuxw/computation/build/count /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/$i/velodyne_${data_name} /home/yuxw/Downloads/count ${data_name}_$i
done
echo "Task completed!"
