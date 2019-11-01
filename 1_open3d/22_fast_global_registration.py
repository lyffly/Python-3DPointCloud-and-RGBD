# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time


def draw_registration_result(source,target,transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0.706,0])
    target_temp.paint_uniform_color([0,0.651,0.929])
    source_temp.transform(transformation)
        
    op3.visualization.draw_geometries([source_temp,target_temp])

def preprocess_point_cloud(pcd,voxel_size):
    print("Downsample with a voxel size %.3f" % voxel_size)
    pcd_down= pcd.voxel_down_sample(voxel_size)
    
    radius_normal = voxel_size *2
    print("Estimate normal with search radius %.3f" % radius_normal)
    pcd_down.estimate_normals(
        op3.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal,
            max_nn=30
        )
    )

    radius_feature = voxel_size *5
    print("Compute FPFH feature with search radius %.3f" % radius_feature)
    pcd_fpfh = op3.registration.compute_fpfh_feature(
        pcd_down,
        op3.geometry.KDTreeSearchParamHybrid(
            radius = radius_feature,
            max_nn=100
        )
    )
    return pcd_down,pcd_fpfh

def prepare_dataset(voxel_size):
    print(" load 2 point clouds and disturb initial pose")
    source = op3.io.read_point_cloud("demodata/ICP/cloud_bin_0.pcd")
    target = op3.io.read_point_cloud("demodata/ICP/cloud_bin_1.pcd")
    trans_init = np.asarray([[0.0,0.0,1.0,0.0],
                            [1.0,0.0,0.0,0.0],
                            [0.0,1.0,0.0,0.0],
                            [0.0,0.0,0.0,1.0]])
    source.transform(trans_init)
    draw_registration_result(source,target,np.identity(4))

    source_down,source_fpfh = preprocess_point_cloud(source,voxel_size)
    target_down,target_fpfh = preprocess_point_cloud(target,voxel_size)
    return source,target,source_down,target_down,source_fpfh,target_fpfh


def execute_global_registration(source_down,target_down,\
    source_fpfh,target_fpfh,voxel_size):
    distance_threshold = voxel_size*1.5
    result = op3.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        distance_threshold,
        op3.registration.TransformationEstimationPointToPoint(False),
        4,
        [op3.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        op3.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
        op3.registration.RANSACConvergenceCriteria(4000000,500)
    )
    return result

# 快速全局配准算法
def execute_fast_global_registration(source_down,target_down,\
    source_fpfh,target_fpfh,voxel_size):
    distance_threshold = voxel_size*0.5
    result = op3.registration.registration_fast_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        op3.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold
        )
    )
    return result

def refine_registration(source,target,source_fpfh,target_fpfh,voxel_size):
    distance_threshold = voxel_size *0.4
    result = op3.registration.registration_icp(
        source,
        target,
        distance_threshold,
        result_ransac.transformation,
        op3.registration.TransformationEstimationPointToPlane()
    )

    return result




if __name__ == "__main__":
    
    voxel_size = 0.05 # 5cm
    source,target,source_down,target_down,source_fpfh,target_fpfh = \
        prepare_dataset(voxel_size)
    
    start = time.time()

    result_ransac = execute_global_registration(
        source_down,target_down,
        source_fpfh,target_fpfh,
        voxel_size
    )
    print("RANSAC time is %.3f " % (time.time()-start))

    draw_registration_result(source_down,target_down,result_ransac.transformation)

    start = time.time()
    result_fast = execute_fast_global_registration(
        source_down,target_down,
        source_fpfh,target_fpfh,
        voxel_size
    )

    print("快速算法耗时 is %.3f " % (time.time()-start))

    draw_registration_result(source_down,target_down,result_fast.transformation)

    
    
    
