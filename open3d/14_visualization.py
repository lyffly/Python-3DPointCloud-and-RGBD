# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt


if __name__ == "__main__":
    #loading ply point cloud, print it and render it
    pcd =op3.io.read_point_cloud("demodata/fragment.ply")
    op3.visualization.draw_geometries([pcd])

    print("一些基本的物体")

    # 画立方体
    mesh_box = op3.geometry.TriangleMesh.create_box(width=1.0,height=1.0,depth=1.0)
    mesh_box.compute_vertex_normals()
    mesh_box.paint_uniform_color([0.9,0.1,0.1])

    # 画球
    mesh_sphere = op3.geometry.TriangleMesh.create_sphere(radius=1.0)
    mesh_sphere.compute_vertex_normals()
    mesh_sphere.paint_uniform_color([0.1,0.1,0.7])

    # 画圆柱
    mesh_cylinder = op3.geometry.TriangleMesh.create_cylinder(
        radius=0.3,height=4.0
    )
    mesh_cylinder.compute_vertex_normals()
    mesh_cylinder.paint_uniform_color([0.1,0.8,0.1])

    # 画frame
    mesh_frame = op3.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6,origin=[-2,-2,-2]
    )
    # 显示
    op3.visualization.draw_geometries([mesh_box,mesh_cylinder,mesh_sphere,mesh_frame])
    op3.visualization.draw_geometries([mesh_box+mesh_cylinder+mesh_sphere+mesh_frame])

    # 点
    points =[[0,0,0],[1,0,0],[0,1,0],[1,1,0],[0,0,1],[1,0,1],[0,1,1],[1,1,1]]
    # 线
    lines =[[0,1],[0,2],[1,3],[2,3],[4,5],[4,6],[5,7],[6,7],[0,4],[1,5],[2,6],[3,7]]
    # 颜色
    colors=[[1,0,0] for i in range(len(lines))]

    line_set = op3.geometry.LineSet()
    line_set.points=op3.utility.Vector3dVector(points)
    line_set.lines = op3.utility.Vector2iVector(lines)
    line_set.colors = op3.utility.Vector3dVector(colors)
    op3.visualization.draw_geometries([line_set])








