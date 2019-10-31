# 3D点云及RGBD 处理学习记录

##### 整理和编码：刘云飞 

##### 编程语言：python

##### 主要库：open3d , pyrealsense 



#### 一、open3d

GitHub :https://github.com/intel-isl/Open3D

Doc: http://www.open3d.org/docs/release/



整理后的例程

| 序号 | 文件名                                 | 描述                        |
| :--: | -------------------------------------- | --------------------------- |
|  1   | 1_read_pcd.py                          | 读取pcd格式的文件           |
|  2   | 2_read_ply.py                          | 读取ply格式的文件           |
|  3   | 3_read_jpg.py                          | 读图片                      |
|  4   | 4_usage_of_pointcloud.py               | 点云的使用                  |
|  5   | 5_crop_pointcloud.py                   | 点云的剪切                  |
|  6   | 6_paint_pointcloud.py                  | 点云着色                    |
|  7   | 7_usage_of_mesh.py                     | 曲面的使用                  |
|  8   | 8_paint_mesh.py                        | 曲面着色                    |
|  9   | 9_read_rgbd.py                         | 读取rgbd数据                |
|  10  | 10_read_sun_rgbd.py                    | 读SUN的数据集               |
|  11  | 11_read_TUM_rgbd.py                    | 读TUM的数据集               |
|  12  | 12_RGBD_odometry.py                    |                             |
|  13  | 13_RGBD_odometry_hybrid.py             |                             |
|  14  | 14_visualization.py                    | 点云可视化                  |
|  15  | 15_KDTree.py                           | KDTree                      |
|  16  | 16_ICP_registration_point2point.py     | 点云迭代匹配 优化点到点距离 |
|  17  | 17_ICP_registration_point2plane.py     | 点云迭代匹配 优化点到面距离 |
|  18  | 18_work_with_numpy.py                  | 和numpy配合使用             |
|  19  | 19_remove_outlier_cloud_point.py       | 去除点云的点，滤波          |
|  20  | 20_colored_point_cloud_registration.py |                             |
|  21  | 21_global_registration.py              | 全部配准算法                |
|  22  | 22_fast_global_registration.py         | 快速全局配准算法            |
|  23  | 23_multiway_registration.py            |                             |
|  24  | 24_rgb_integration.py                  |                             |
|  25  | 25_color_map_optiization.py            |                             |
|  26  | 26_customized_visualization.py         | 用户定制可视化              |
|  27  | 27_non_blocking_visualization.py       | 非阻塞可视化                |
|  28  | 28_iteractive_visualization.py         | 带交互可视化                |

ICP demo:

![image](imgs/icp_demo.gif)

#### 二、realsense camera

Web: https://software.intel.com/zh-cn/realsense

Github:https://github.com/IntelRealSense/librealsense

Python Example: 

https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples

整理的例程

| 序号 | 文件名                     | 描述                      |
| :--: | -------------------------- | ------------------------- |
|  1   | 1_read_depth.py            | 获取相机的深度数据        |
|  2   | 2_read_rgbd_with_config.py | 获取相机的RGBD数据        |
|  3   | 3_align_depth_to_color.py  | 深度数据和颜色数据对齐    |
|  4   | 4_realsense_with_open3d.py | Realsense和open3d组合使用 |
|  5   | 5_realsense_imu.py         | IMU数据读取               |

Demo视频：

 ![image]( imgs/demo.gif)

#### 三、transform3d

Github:  https://github.com/matthew-brett/transforms3d

Doc: http://matthew-brett.github.io/transforms3d



#### 四、常用的相机厂家整理

见文件夹



