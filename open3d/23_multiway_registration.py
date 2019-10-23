# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time


voxel_size = 0.02
max_correspondense_distance_coarse = voxel_size*15
max_correspondense_distance_fine = voxel_size *1.5

def load_point_clouds(voxel_size = 0.0):
    pcds = []
    for i in range(3):
        pcd = op3.io.read_point_cloud("demodata/ICP/cloud_bin_%d.pcd" % i)
        pcd_down = pcd.voxel_down_sample(voxel_size = voxel_size)
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source,target):
    print("进行 点到面 ICP")
    icp_coarse = op3.registration.registration_icp(
        source,
        target,
        max_correspondense_distance_coarse,
        np.identity(4),
        op3.registration.TransformationEstimationPointToPlane()
    )
    icp_fine = op3.registration.registration_icp(
        source,
        target,
        max_correspondense_distance_fine,
        icp_coarse.transformation,
        op3.registration.TransformationEstimationPointToPlane()
    )
    transformation_icp = icp_fine.transformation
    information_icp = op3.registration.get_information_matrix_from_point_clouds(
        source,
        target,
        max_correspondense_distance_fine,
        icp_fine.transformation
    )
    return transformation_icp,information_icp


def full_registration(pcds,max_correspondense_distance_coarse,max_correspondense_distance_fine):
    pose_graph = op3.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(op3.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id+1,n_pcds):
            transformation_icp,information_icp = pairwise_registration(
                pcds[source_id],pcds[target_id]
            )
            print("build op3.registration.PoseGraph")
            if target_id ==source_id +1: #odometry case
                odometry = np.dot(transformation_icp,odometry)
                pose_graph.nodes.append(
                    op3.registration.PoseGraphNode(np.linalg.inv(odometry))
                )
                pose_graph.edges.append(
                    op3.registration.PoseGraphEdge(
                        source_id,
                        target_id,
                        transformation_icp,
                        information_icp,
                        uncertain = False
                    )
                )
            else:
                pose_graph.edges.append(
                    op3.registration.PoseGraphEdge(
                        source_id,
                        target_id,
                        transformation_icp,
                        information_icp,
                        uncertain = True
                    )
                )
    return pose_graph



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
    
    op3.utility.set_verbosity_level(op3.utility.VerbosityLevel.Debug)
    pcds_down = load_point_clouds(voxel_size)
    op3.visualization.draw_geometries(pcds_down)


    print("Full registration ")

    pose_graph = full_registration(pcds_down,max_correspondense_distance_coarse,max_correspondense_distance_fine)

    print("optimizing PoseGraph")
    option = op3.registration.GlobalOptimizationOption(
        max_correspondence_distance= max_correspondense_distance_fine,
        edge_prune_threshold = 0.25,
        reference_node =0
    )
    op3.registration.global_optimization(
        pose_graph,
        op3.registration.GlobalOptimizationLevenbergMarquardt(),
        op3.registration.GlobalOptimizationConvergenceCriteria(),
        option
    )

    
    print("transform points and display ")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)

    op3.visualization.draw_geometries(pcds_down)

    print("Make a combined point cloud")
    pcds = load_point_clouds(voxel_size)
    pcd_combined = op3.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    op3.io.write_point_cloud("multiway_registration.pcd",pcd_combined_down)

    op3.visualization.draw_geometries([pcd_combined_down])

    

    
    
    
