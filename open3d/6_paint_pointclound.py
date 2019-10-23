# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import open3d as o3d
import numpy as np
import copy


def read_and_show():
    vol = o3d.visualization.read_selection_polygon_volume("demodata/cropped.json")
    pcd  = o3d.io.read_point_cloud("demodata/fragment.ply")
    
    chair = vol.crop_point_cloud(pcd)
    return chair

def paint_cloud(pcd):
    pcd.paint_uniform_color([1,0.706,0])
    return pcd

if __name__ == "__main__":
    chair  = read_and_show()
    chair = paint_cloud(chair)

    o3d.visualization.draw_geometries([chair])
 

    
    

