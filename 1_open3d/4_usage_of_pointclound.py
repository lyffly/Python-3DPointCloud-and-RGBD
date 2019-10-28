# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import open3d as op3
import numpy as np
import copy


def read_and_show():
    pcd  = op3.io.read_point_cloud("demodata/fragment.ply")
 
    return pcd

def downshape(pcd):
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    return downpcd

def compute_normal_of_clound(pcd):
    pcd2 = copy.deepcopy(pcd)
    pcd2.estimate_normals(search_param=op3.geometry.KDTreeSearchParamHybrid(
        radius=0.1,max_nn=30))
    #op3.visualization.draw_geometries([pcd2])
    return pcd2

if __name__ == "__main__":
    pcd = read_and_show()
    downpcd =  downshape(pcd)

    op3.visualization.draw_geometries([pcd])
    op3.visualization.draw_geometries([downpcd])

    downpcd2 = compute_normal_of_clound(downpcd)
    op3.visualization.draw_geometries([downpcd2])
    print(downpcd.normals[0])
    print(downpcd2.normals[0])
    

