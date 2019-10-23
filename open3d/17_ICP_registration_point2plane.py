# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time


def draw_registration_result(source,target,transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0.706,0])
    target_temp.paint_uniform_color([0,0.651,0.929])

    source_temp.transform(transformation)
    op3.visualization.draw_geometries([source_temp,target_temp])

if __name__ == "__main__":
    # testing ketree in open3d
    # load a point cloud and paint it gray
    source =op3.io.read_point_cloud("demodata/ICP/cloud_bin_0.pcd")
    target =op3.io.read_point_cloud("demodata/ICP/cloud_bin_1.pcd")
    threshold=0.02
    trans_init=np.asarray([[0.862,0.011,-0.507,0.5],
                        [-0.139,0.967,-0.215,0.7],
                        [0.487,0.255,0.835,-1.4],
                        [0.0,0.0,0.0,1.0]])
    
    draw_registration_result(source,target,trans_init)
    #initial alignment

    evaluation = op3.registration.evaluate_registration(
        source,
        target,
        threshold,
        trans_init
    )
    print(evaluation)

    t1 = time.time()
# point to plane ICP
    reg_p2l = op3.registration.registration_icp(
        source,
        target,
        threshold,
        trans_init,
        op3.registration.TransformationEstimationPointToPlane(),
        op3.registration.ICPConvergenceCriteria(max_iteration=200)
        )
    print(reg_p2l)
    print("transformation is ")
    print(reg_p2l.transformation)

    dt = time.time()-t1
    print(dt)

    draw_registration_result(source,target,reg_p2l.transformation)







