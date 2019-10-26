# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import open3d as op3
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # 针孔相机的内参
    pinhole_camera_intrinsic =op3.io.read_pinhole_camera_intrinsic("demodata/camera_primesense.json")
    print(pinhole_camera_intrinsic.intrinsic_matrix)

    # 读两幅图的 RGB和depth图
    source_color = op3.io.read_image('demodata/RGBD/color/00000.jpg')
    source_depth = op3.io.read_image('demodata/RGBD/depth/00000.png')
    target_color = op3.io.read_image('demodata/RGBD/color/00001.jpg')
    target_depth = op3.io.read_image('demodata/RGBD/depth/00001.png')
    
    # source image 原始图
    source_rgbd_image = op3.geometry.RGBDImage.create_from_color_and_depth(
        source_color,source_depth
    )
    # target image 目标图
    target_rgbd_image = op3.geometry.RGBDImage.create_from_color_and_depth(
        target_color,target_depth
    )

    # 创建点云
    target_pcd = op3.geometry.PointCloud.create_from_rgbd_image(
        target_rgbd_image,
        pinhole_camera_intrinsic
        )
    # 里程计参数选项
    option = op3.odometry.OdometryOption()
    # 初始变换矩阵
    odo_init = np.identity(4)
    print(option)

    #计算从source到target 的变换矩阵
    [success_color_term,trans_color_term,info] = op3.odometry.compute_rgbd_odometry(
        source_rgbd_image,
        target_rgbd_image,
        pinhole_camera_intrinsic,
        odo_init,
        op3.odometry.RGBDOdometryJacobianFromColorTerm(),
        option
    )


    if success_color_term:
        print("using RGB-D Odometry \n 使用RGBD里程计计算：")
        print(trans_color_term)
        source_pcd_color_term = op3.geometry.PointCloud.create_from_rgbd_image(
            source_rgbd_image,pinhole_camera_intrinsic
        )

        source_pcd_color_term.transform(trans_color_term)
        
        op3.visualization.draw_geometries([target_pcd,source_pcd_color_term])

    






