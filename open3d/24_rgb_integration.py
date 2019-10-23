# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time
from trajectory_io import *



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
    camera_poses = read_trajectory("demodata/RGBD/odometry.log")
    volume = op3.integration.ScalableTSDFVolume(
        voxel_length = 4.0/512.0,
        sdf_trunc=0.04,
        color_type = op3.integration.TSDFVolumeColorType.RGB8
    )

    for i in range(len(camera_poses)):
        print("integrate {:d} -th image into volume".format(i))
        color = op3.io.read_image("demodata/RGBD/color/{:05d}.jpg".format(i))
        depth = op3.io.read_image("demodata/RGBD/depth/{:05d}.png".format(i))
        rgbd = op3.geometry.RGBDImage.create_from_color_and_depth(
            color,depth,
            depth_trunc=4.0,
            convert_rgb_to_intensity=False    
        )
        volume.integrate(
            rgbd,
            op3.camera.PinholeCameraIntrinsic(
                op3.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
            np.linalg.inv(camera_poses[i].pose)            
        )
    print("Extract a triangle mesh from the volume and visualize it ")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    op3.visualization.draw_geometries([mesh])

    
    
