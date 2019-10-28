# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time
from trajectory_io import *
import os
import sys


def demo_crop_geometry():
    print("Demo for manual geometry cropping")
    print("1. press Y twice to align geometry with negative derection of y-axis")
    print("2. press K to lock screen and to switch to selection mode")
    print("3. Drag for rectangle selection")
    print("     or use ctrl+left click for polygon selection")
    print("4. press C to get a selected geometry and to save it")
    print("5. press F to switch to freeview mode")
    pcd = op3.io.read_point_cloud("demodata/ICP/cloud_bin_0.pcd")
    op3.visualization.draw_geometries_with_editing([pcd])
    
def draw_registration_result(source,target,transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0.706,0])
    target_temp.paint_uniform_color([0,0.651,0.929])
    source_temp.transform(transformation)
    op3.visualization.draw_geometries([source_temp,target_temp])
    


def pick_points(pcd):
    print("")
    print(
        "1."
    )
    vis = op3.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    return vis.get_picked_points()

def demo_manual_registration():
    source = op3.io.read_point_cloud("demodata/ICP/cloud_bin_0.pcd")
    target = op3.io.read_point_cloud("demodata/ICP/cloud_bin_2.pcd")
    draw_registration_result(source,target,np.identity(4))

    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert (len(picked_id_source)>=3 and len(picked_id_target)>=3)
    assert (len(picked_id_source)==len(picked_id_target))
    corr = np.zeros((len(picked_id_source),2))
    corr[:,0] = picked_id_source
    corr[:,1] = picked_id_target

    p2p =op3.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source,target,op3.utility.Vector2iVector(corr))

    threshold =0.03
    reg_p2p = op3.registration.registration_icp(
        source,target,
        threshold,trans_init,
        op3.registration.TransformationEstimationPointToPoint()
    )
    draw_registration_result(source,target,reg_p2p.transformation)


if __name__ == "__main__":
    demo_crop_geometry()
    demo_manual_registration()




