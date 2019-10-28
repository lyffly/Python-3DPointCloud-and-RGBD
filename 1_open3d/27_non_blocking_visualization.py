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




if __name__ == "__main__":
    op3.utility.set_verbosity_level(op3.utility.VerbosityLevel.Debug)

    source_raw = op3.io.read_point_cloud("demodata/ICP/cloud_bin_0.pcd")
    target_raw = op3.io.read_point_cloud("demodata/ICP/cloud_bin_1.pcd")

    source = source_raw.voxel_down_sample(voxel_size = 0.02)
    target = target_raw.voxel_down_sample(voxel_size = 0.02)

    trans = [[0.862,0.011,-0.507,0.0],
            [-0.139,0.967,-0.215,0.7],
            [0.487,0.255,0.835,-1.4],
            [0.0,0.0,0.0,1.0]]
    source.transform(trans)

    flip_tranform = [[1,0,0,0],
                    [0,-1,0,0],
                    [0,0,-1,0],
                    [0,0,0,1]]

    source.transform(flip_tranform)
    target.transform(flip_tranform)

    vis =op3.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(source)
    vis.add_geometry(target)
    threshold =0.05
    icp_iteration =200
    save_image = False
    for i in range(icp_iteration):
        reg_p2l = op3.registration.registration_icp(
            source,
            target,
            threshold,
            np.identity(4),
            op3.registration.TransformationEstimationPointToPlane(),
            op3.registration.ICPConvergenceCriteria(max_iteration=1)
        )
        source.transform(reg_p2l.transformation)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" %i)
    vis.destroy_window()


