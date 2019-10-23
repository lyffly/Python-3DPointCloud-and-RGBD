# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import open3d as op3
import numpy as np
import matplotlib.pyplot as plt


filename_color = 'demodata/RGBD/other_formats/TUM_color.png'
filename_depth = 'demodata/RGBD/other_formats/TUM_depth.png'


def plot_2_images(img1,img2,str1,str2):
    plt.subplot(1,2,1)
    plt.title(str1)
    plt.imshow(img1)
    plt.subplot(1,2,2)
    plt.title(str2)
    plt.imshow(img2)
    plt.show()


if __name__ == "__main__":
    
    #read sun datasets
    color_raw = op3.io.read_image(filename_color)
    depth_raw = op3.io.read_image(filename_depth)
    
    rgbd_image = op3.geometry.RGBDImage.create_from_tum_format(
        color_raw,depth_raw
    )
    print(rgbd_image)


    plt.subplot(1,2,1)
    plt.title("TUM grayscale image")
    plt.imshow(rgbd_image.color)
    plt.subplot(1,2,2)
    plt.title("depth image")
    plt.imshow(rgbd_image.depth)
    plt.show()

    pcd = op3.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        op3.camera.PinholeCameraIntrinsic(op3.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        )
    
    pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    op3.visualization.draw_geometries([pcd])






