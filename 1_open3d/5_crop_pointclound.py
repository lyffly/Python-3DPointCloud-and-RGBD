# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import open3d as op3
import numpy as np
import copy


def read_and_show():
    vol = op3.visualization.read_selection_polygon_volume("demodata/cropped.json")
    pcd  = op3.io.read_point_cloud("demodata/fragment.ply")
    
    chair = vol.crop_point_cloud(pcd)
    return chair

if __name__ == "__main__":
    chair  = read_and_show()    

    op3.visualization.draw_geometries([chair])
 

    
    

