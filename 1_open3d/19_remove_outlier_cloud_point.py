# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time


def display_inlier_outlier(cloud,ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind,invert=True)

    print("红色在外面，灰色在里面")
    
    outlier_cloud.paint_uniform_color([1,0,0])
    inlier_cloud.paint_uniform_color([0.8,0.8,0.8])
    op3.visualization.draw_geometries([inlier_cloud,outlier_cloud])


if __name__ == "__main__":
    
    #读 点云数据
    pcd = op3.io.read_point_cloud("demodata/ICP/cloud_bin_2.pcd")
    op3.visualization.draw_geometries([pcd])


    #使用0.02的体素对点云进行下采样
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    op3.visualization.draw_geometries([voxel_down_pcd])

    #每隔5个点采样
    uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
    op3.visualization.draw_geometries([uni_down_pcd])
    
    #统计异常值去除
    cl,ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,std_ratio=2.0)

    display_inlier_outlier(voxel_down_pcd,ind)


    #半径异常之去除
    cl,ind = voxel_down_pcd.remove_radius_outlier(nb_points=16,radius=0.05)

    display_inlier_outlier(voxel_down_pcd,ind)

