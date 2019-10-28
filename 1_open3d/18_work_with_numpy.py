# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time

if __name__ == "__main__":
    
    x = np.linspace(-3,3,401)
    mesh_x,mesh_y = np.meshgrid(x,x)
    z = np.sinc((np.power(mesh_x,2)+np.power(mesh_y,2)))
    z_norm = (z-z.min())/(z.max()-z.min())
    xyz = np.zeros((np.size(mesh_x),3))
    xyz[:,0] = np.reshape(mesh_x,-1) 
    xyz[:,1] = np.reshape(mesh_y,-1) 
    xyz[:,2] = np.reshape(z_norm,-1) 
    print("xyz")
    print(xyz)

    pcd = op3.geometry.PointCloud()
    pcd.points = op3.utility.Vector3dVector(xyz)
    op3.io.write_point_cloud("demodata/testxyz.ply",pcd)

    pcd_load = op3.io.read_point_cloud("demodata/testxyz.ply")
    op3.visualization.draw_geometries([pcd_load])

    xyz_load = np.asarray(pcd.points)
    print("xyz load")
    print(xyz_load)

    img = op3.geometry.Image((z_norm*255).astype(np.uint8))
    op3.io.write_image("demodata/testxyz.jpg",img)
    op3.visualization.draw_geometries([img])
    

