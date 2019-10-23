# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time


def draw_registration_result_origin_color(source,target,transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
        
    op3.visualization.draw_geometries([source_temp,target])


if __name__ == "__main__":
    
    #1. 读两种点云数据，现实初始姿态

    source = op3.io.read_point_cloud("demodata/ColoredICP/frag_115.ply")
    target = op3.io.read_point_cloud("demodata/ColoredICP/frag_116.ply")

    #op3.visualization.draw_geometries([source])
    #op3.visualization.draw_geometries([target])
    # 画初始的位置
    current_transformation = np.identity(4)
    draw_registration_result_origin_color(source,target,current_transformation)
    
    # point-to-plane ICP
    current_transformation = np.identity(4)
    # 2. 点到面 ICP 
    result_icp = op3.registration.registration_icp(
        source,
        target,
        0.02,
        current_transformation,
        op3.registration.TransformationEstimationPointToPlane()
    )
    print(result_icp)
    draw_registration_result_origin_color(source,target,result_icp.transformation)

    # colored pointcloud registration
    voxel_radius = [0.04,0.02,0.01]
    max_iter=[50,30,14]
    current_transformation = np.identity(4)
    # 3. colored pointcloud registration
    
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter,radius,scale])

        print("3.1 downsample with a voxel size of %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3.2 Estimate normal.")
        source_down.estimate_normals(
            op3.geometry.KDTreeSearchParamHybrid(radius=radius*2,max_nn=30)
        )
        target_down.estimate_normals(
            op3.geometry.KDTreeSearchParamHybrid(radius=radius*2,max_nn=30)
        )

        print("3.3 Applying colored point cloud registration")
        result_icp = op3.registration.registration_colored_icp(
            source_down,
            target_down,
            radius,
            current_transformation,
            op3.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=iter
            )

        )
        current_transformation = result_icp.transformation

        print(result_icp)
    draw_registration_result_origin_color(source,target,result_icp.transformation)


    
    
