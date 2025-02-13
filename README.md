# AVS

This is a ros2 humble package to achieve the autonomous vehicle storage functionality.

## Build
### ros2 packages
```
git clone https://github.com/Ariiees/AVS.git
cd AVS
colcon build
```
### computation
```
cd AVS/src/computation
mkdir build
cd build
cmake ..
make
```

## launch 
### ros2 packages
```
ros2 launch ros2bag_write_benchmark benchmarker_launch.py
ros2 run avedr edr
ros2 run pcl_filter pcl_filter_node
```
### computation
```
./computation /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/10/velodyne_old /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/10/velodyne
./count /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/10/velodyne_vs03only /home/yuxw/Downloads/count vs03only_10
python visualize.py /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/00/velodyne_old/000000.bin /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/00/velodyne_vs01only/000000.bin
```
