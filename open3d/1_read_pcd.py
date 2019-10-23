# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import time


if __name__ == "__main__":

    #打开pcd文件 point cloud
    pcd = op3.io.read_point_cloud('demodata/fragment.pcd')
    print(pcd)

    #显示
    op3.visualization.draw_geometries([pcd])

    #保存文件
    op3.io.write_point_cloud("demodata/pcd_1.pcd",pcd)

