# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)


import numpy as np
import open3d as op3
import matplotlib.pyplot as plt

filename_color = 'demodata/RGBD/color/00000.jpg'
filename_depth = 'demodata/RGBD/depth/00000.png'


def plot_2_images(img1,img2,str1,str2):
    """
    plot 2 images in 1 window using matplotlib
    
    args:
        img1: image1 numpy format
        img2: image2 numpy format
        str1:image 1 title
        str2:image 2 title
    
    returns:
        None
    """
    plt.subplot(1,2,1)
    plt.title(str1)
    plt.imshow(img1)
    plt.subplot(1,2,2)
    plt.title(str2)
    plt.imshow(img2)
    plt.show()


if __name__ == "__main__":
    # color 和depth 数据读取
    color_raw = op3.io.read_image(filename_color)
    depth_raw = op3.io.read_image(filename_depth)
    # 由color和depth数据生成rgbd 数据
    rgbd_image = op3.geometry.RGBDImage.create_from_color_and_depth(
        color_raw,depth_raw
    )
    print(rgbd_image)
    # 显示在窗口中
    plot_2_images(rgbd_image.color,rgbd_image.depth,"grayscale image","depth image")
    # 由rgbd数据 生成 点云数据
    pcd = op3.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        op3.camera.PinholeCameraIntrinsic(op3.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        )
    # 数据平移和反转，这里反转
    pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    # 点云数据显示
    op3.visualization.draw_geometries([pcd])






