import os
import numpy as np
import struct
import open3d as o3d

def read_bin_velodyne(path):
    pc_list = []
    with open(path, 'rb') as f:
        content = f.read()
        pc_iter = struct.iter_unpack('ffff', content)
        for point in pc_iter:
            pc_list.append([point[0], point[1], point[2]])
        print(len(pc_list))
    return np.asarray(pc_list, dtype=np.float32)

def main():
    import sys
    if len(sys.argv) != 2:
        print("Usage: python visualize_pointcloud.py <pointcloud1.bin> <pointcloud2.bin>")
        sys.exit(1)

    file1 = sys.argv[1]
    # file2 = sys.argv[2]

    # if not os.path.isfile(file1) or not os.path.isfile(file2):
    #     print("One or both file paths are invalid.")
    #     sys.exit(1)

    # print(f"Comparing files:\n  File 1: {file1}\n  File 2: {file2}")

    # Load the point clouds
    pc1 = read_bin_velodyne(file1)
    # pc2 = read_bin_velodyne(file2)

    # Convert numpy arrays to Open3D PointCloud objects
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pc1)
    pcd1.paint_uniform_color([1, 0, 0])  # [R,G,B]
    # pcd2 = o3d.geometry.PointCloud()
    # pcd2.points = o3d.utility.Vector3dVector(pc2)
    # pcd2.paint_uniform_color([0, 0, 1])  

    # Visualize the two point clouds together
    # o3d.visualization.draw_geometries([pcd1, pcd2],
    #                                   window_name="Comparison of Two Point Clouds",
    #                                   width=800,
    #                                   height=600)
    
    o3d.visualization.draw_geometries([pcd1],
                                      window_name="visualize",
                                      width=800,
                                      height=600)

if __name__ == "__main__":
    main()