# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3



if __name__ == "__main__":
    
    # 读ply文件
    mesh = op3.io.read_triangle_mesh('demodata/knot.ply')
    print(mesh)

    #计算其法向量
    mesh.compute_vertex_normals()

    #显示 mesh
    op3.visualization.draw_geometries([mesh])
    print("OK")



