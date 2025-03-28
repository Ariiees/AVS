# AVS

This is a ROS2 humble package with CPP and python script to achieve the autonomous vehicle storage functionality.

## Function
- **ros2 bag write benchmark:** It benchmarks the ros2 bag stored in MCAP and SQLite write performance, containing the write latency, throughput, and data loss.
  
  Test: Test with Camera(Basler ace acA1920-40gc) and Lidar(Hesai Pandar 64) data in real time.
  
  Results: Image data writing improved by 34%, and LiDAR data writing improved by 27%.
  
- **avder (black box):** It store 20s pre crash and 10s post crash data.
  
  Test: Test on BlueICE simulation to store camera image, lidar point cloud, and vehicle status CAN BUS data.

  Results: Could store sucessfuly without delay and data lost, the data could be play back (video is in [here](https://drive.google.com/file/d/1HzeM2vFeyduPrTIPICIipA0ToyzfAI4Z/view?usp=sharing)).
  
- **lidar frame downsample:** It aims to show that not all data are needed for localization-related tasks (mainly what task lidar works for).

  Test: As a case study, we use a voxel grid filter to downsample the KITTI lidar data set gradually and then feed it to [KISS-ICP](https://github.com/PRBonn/kiss-icp) to see at which level the KISS-ICP will lose register and localization ability.

  Results: Downsample the original data size to only 24.38% and keep the KISS-ICP performance to 1.7493 m for absolute trajectory error and 0.0014 deg/m for average rotation error.
  
- **lidar frame deduplicate:** Lidar will generate data in 10Hz, but not all frames are needed; some contain duplicated information. This function will only store the KEY FRAME in the real-time sequence of lidar data.

  Test: Test with real-time Hesai Pandar 64 lidar point cloud data.

  Results: Drive in Udel STAR Camp with 10 mph, lidar generates 1238 frames, only 169 frames are recorded. Drive around Udel modular lab, lidar generates 999 frames, only 66 frames are recorded. Stay in parkking lot, but other vehicle move around, lidar generates 656 frames, and only 22 key frames are recorded. Vedios are showing [here](https://drive.google.com/drive/folders/1gw72LfTRhE1ekn5KRpBqDVP69j3kucNV?usp=sharing).
  
## Build
### ros2 packages
```
git clone https://github.com/Ariiees/AVS.git
cd AVS
colcon build
```
### lidar frame downsample
```
cd AVS/src/lidar_frame_downsample
mkdir build
cd build
cmake ..
make
```

## Launch
### ros2 packages
```
ros2 launch ros2bag_write_benchmark benchmarker_launch.py
ros2 run avedr edr
```
### lidar frame downsample
```
./computation /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/10/velodyne_old /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/10/velodyne
./count /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/10/velodyne_vs03only /home/yuxw/Downloads/count vs03only_10
python visualize.py /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/00/velodyne_old/000000.bin /home/yuxw/Downloads/data/kitti-odometry/dataset/sequences/00/velodyne_vs01only/000000.bin
```
### lidar frame deduplicate
```
cd AVS/src/lidar_frame_deduplicate/script
python3 frame_deduplicate.py
```
