# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import open3d as op3
import numpy as np
import copy


if __name__ == "__main__":
    
    mesh = op3.io.read_triangle_mesh("demodata/knot.ply")
    print(mesh)
    print(np.asarray(mesh.vertices))
    print(np.asarray(mesh.triangles))
    print("")

    print("vertex exist " + str(mesh.has_vertex_normals()) + " corlor exist "+str(mesh.has_vertex_normals()))
    op3.visualization.draw_geometries([mesh])
    print("没有法相和颜色的图")

    #计算法向量
    mesh.compute_vertex_normals()
    op3.visualization.draw_geometries([mesh])

    mesh1 = copy.deepcopy(mesh)
    mesh1.triangles = op3.utility.Vector3iVector(
        np.asarray(mesh1.triangles)[:len(mesh1.triangles)//2,:]
     )
    mesh1.triangle_normals = op3.utility.Vector3dVector(
        np.asarray(mesh1.triangle_normals)[:len(mesh1.triangle_normals)//2,:]
     )
    print(mesh1.triangles)
    op3.visualization.draw_geometries([mesh1])


    