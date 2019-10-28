# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # testing ketree in open3d
    # 读数据
    pcd =op3.io.read_point_cloud("demodata/Feature/cloud_bin_0.pcd")

    # 涂成灰色
    pcd.paint_uniform_color([0.5,0.5,0.5])
    # KD tree
    pcd_tree = op3.geometry.KDTreeFlann(pcd)
    op3.visualization.draw_geometries([pcd])

    
    # 把第1500个点，变成红色
    pcd.colors[1500] =[1,0,0]

    # 找到最近的200个点，变成蓝色
    [k,idx,_]= pcd_tree.search_knn_vector_3d(pcd.points[1500],200)
    np.asarray(pcd.colors)[idx[1:],:] = [0,0,1]

    # 找到距离小于0.2的临近点 变成绿色
    [k,idx,_]= pcd_tree.search_radius_vector_3d(pcd.points[1500],0.2)
    np.asarray(pcd.colors)[idx[1:],:] = [0,1,0]


    # 可视化 点云
    op3.visualization.draw_geometries([pcd])
    








