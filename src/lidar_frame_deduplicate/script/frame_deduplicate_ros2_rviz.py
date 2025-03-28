import os
import numpy as np
import open3d as o3d
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
from tqdm import tqdm

# ==============================================================================
# Utility Functions
# ==============================================================================
def read_bin_file(filename):
    points = np.fromfile(filename, dtype=np.float32).reshape(-1, 4)
    return points[:, :3]  # Extract only x, y, z

def read_pcd_file(filename):
    pcd = o3d.io.read_point_cloud(filename)
    return np.asarray(pcd.points)

def load_point_cloud(file_path):
    ext = os.path.splitext(file_path)[-1].lower()
    if ext == ".bin":
        return read_bin_file(file_path)
    elif ext == ".pcd":
        return read_pcd_file(file_path)
    else:
        raise ValueError(f"Unsupported file format: {ext}")
    
# ==============================================================================
# Projection Functions
# ==============================================================================
def scale_to_255(value, min, max, dtype=np.uint8):
    return (((value - min) / float(max - min)) * 255).astype(dtype)

def birds_eye_point_cloud(points, xy_range=(-20, 20), z_range = (-1.2, 1), res=0.1):
    x_lidar, y_lidar, z_lidar = points[:, 0], points[:, 1], points[:, 2]
    
    ff = np.logical_and((x_lidar > xy_range[0]), (x_lidar < xy_range[1]))
    ss = np.logical_and((y_lidar > xy_range[0]), (y_lidar < xy_range[1]))
    indices = np.argwhere(np.logical_and(ff, ss)).flatten()

    #Hesai pandar 64 sensor coord to cv2 image coord
    x_img = (x_lidar[indices] / res).astype(np.int32)
    y_img = (y_lidar[indices] / res).astype(np.int32)
    x_img = x_img + abs(np.min(x_img))
    y_img = y_img + abs(np.min(y_img))
    x_max = int((xy_range[1] - xy_range[0]) / res)
    y_max = int((xy_range[1] - xy_range[0]) / res)
    x_img = -x_img + x_max - 1 

    pixel_values = np.clip(z_lidar[indices], z_range[0], z_range[1])
    pixel_values = scale_to_255(pixel_values, min=z_range[0], max=z_range[1])
    im = np.zeros([x_max, y_max], dtype=np.uint8)

    im[y_img, x_img] = pixel_values

    return im
    
# ==============================================================================
# Gray Image Histrogram Comparison
# ==============================================================================
def compare_hist(img1, img2):
    hist_img1 = cv2.calcHist([img1], [0], None, [256], [0, 256])
    cv2.normalize(hist_img1, hist_img1, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    hist_img2 = cv2.calcHist([img2], [0], None, [256], [0, 256])
    cv2.normalize(hist_img2, hist_img2, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)

    metric_val = cv2.compareHist(hist_img1, hist_img2, cv2.HISTCMP_CHISQR)
    return metric_val

# ==============================================================================
# ROS 2 Node for Publishing Point Clouds
# ==============================================================================
class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/AVS/lidar/deframe/pointcloud', 1)

    def publish_pointcloud(self, points):
        msg = self.convert_to_ros2_msg(points)
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published PointCloud2 message")

    def convert_to_ros2_msg(self, points):
        """ Convert numpy array (N, 3) to ROS 2 PointCloud2 message """
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Change based on your RViz setup

        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 3 float32 values * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join([struct.pack('fff', *p) for p in points])

        return msg

# ==============================================================================
# Main Processing Logic with ROS 2 Publishing
# ==============================================================================
def main(folder):
    rclpy.init()
    publisher = PointCloudPublisher()

    files = sorted([f for f in os.listdir(folder) if f.endswith(".bin") or f.endswith(".pcd")])
    file_paths = [os.path.join(folder, f) for f in files]

    pcd1_points = load_point_cloud(file_paths[0])
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pcd1_points)
    
    publisher.publish_pointcloud(pcd1_points) 
    bev1 = birds_eye_point_cloud(pcd1_points)

    for i in tqdm(range(len(file_paths) - 1), desc="Processing Files", unit="pair"):
        pcd2_points = load_point_cloud(file_paths[i+1])
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(pcd2_points)

        bev2 = birds_eye_point_cloud(pcd2_points)

        hist_chisqr = compare_hist(bev1, bev2)
        if hist_chisqr > 0.003:
            bev1 = bev2
            publisher.publish_pointcloud(pcd2_points)

    rclpy.shutdown()

# ==============================================================================
# Main Entry Point
# ==============================================================================
if __name__ == "__main__":
    folder = "/home/yuxw/Downloads/data/stay_other_move/"  # Change to your folder
    main(folder)
