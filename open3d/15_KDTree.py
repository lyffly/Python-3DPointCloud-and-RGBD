# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # testing ketree in open3d
    # load a point cloud and paint it gray
    pcd =op3.io.read_point_cloud("demodata/Feature/cloud_bin_0.pcd")

    pcd.paint_uniform_color([0.5,0.5,0.5])
    pcd_tree = op3.geometry.KDTreeFlann(pcd)
    op3.visualization.draw_geometries([pcd])

    #paint the 1500th point red
    pcd.colors[1500] =[1,0,0]

    # find its 200 nearest neighbors ,paint blue
    [k,idx,_]= pcd_tree.search_knn_vector_3d(pcd.points[1500],200)
    np.asarray(pcd.colors)[idx[1:],:] = [0,0,1]

    # find its neighbors with distance less than 0.2 ,paint red
    [k,idx,_]= pcd_tree.search_radius_vector_3d(pcd.points[1500],0.2)
    np.asarray(pcd.colors)[idx[1:],:] = [0,1,0]


    #visualize the point cloud
    op3.visualization.draw_geometries([pcd])
    








