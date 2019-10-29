# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time
from trajectory_io import *


if __name__ == "__main__":
    # 读取相机的里程计数据
    camera_poses = read_trajectory("demodata/RGBD/odometry.log")
    # 建立TSDF 
    volume = op3.integration.ScalableTSDFVolume(
        voxel_length = 4.0/512.0, # 4.0m
        sdf_trunc= 0.04,
        color_type = op3.integration.TSDFVolumeColorType.RGB8
    )
    # camera_pose 5
    for i in range(len(camera_poses)):
        print("integrate {:d} -th image into volume".format(i))
        # 读rgb和depth 并生成rgbd 数据
        color = op3.io.read_image("demodata/RGBD/color/{:05d}.jpg".format(i))
        depth = op3.io.read_image("demodata/RGBD/depth/{:05d}.png".format(i))
        rgbd = op3.geometry.RGBDImage.create_from_color_and_depth(
            color,depth,
            depth_trunc=4.0,
            convert_rgb_to_intensity=False    
        )
        # 把新的rgbd 集成
        volume.integrate(
            rgbd,
            op3.camera.PinholeCameraIntrinsic(
                op3.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
            np.linalg.inv(camera_poses[i].pose)            
        )
    print("把volume的数据转换为mesh，并显示出来")
    mesh = volume.extract_triangle_mesh()
    # 计算法向量
    mesh.compute_vertex_normals()
    op3.visualization.draw_geometries([mesh])
    # volume数据计算出 点云数据
    pcd = volume.extract_point_cloud()
    op3.visualization.draw_geometries([pcd])

    
    
