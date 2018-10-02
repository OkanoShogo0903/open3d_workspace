# -*- coding: utf-8 -*-
import numpy as np
import copy
from open3d import *

if __name__ == "__main__":
    print("Testing mesh in py3d ...")
    mesh = read_triangle_mesh("./TestData/knot.ply")
    print(mesh)
    print(np.asarray(mesh.vertices)) # vertices : 頂点(複数形)
    print(np.asarray(mesh.triangles)) # 三角形
    print("")

    # メッシュの可視化
    print("Try to render a mesh with normals (exist: " +
        str(mesh.has_vertex_normals()) +
        ") and colors (exist: " + str(mesh.has_vertex_colors()) + ")")
    draw_geometries([mesh])
    print("A mesh with no normals and no colors does not seem good.")

    # 表面から法線を推定して可視化
    print("Computing normal and rendering it.")
    mesh.compute_vertex_normals() # vertex : 頂点
    print(np.asarray(mesh.triangle_normals))
    draw_geometries([mesh])

    # データを半分にして再度描写
    print("We make a partial mesh of only the first half triangles.")
    mesh1 = copy.deepcopy(mesh)
    mesh1.triangles = Vector3iVector(
        np.asarray(mesh1.triangles)[:len(mesh1.triangles)//2, :])
    mesh1.triangle_normals = Vector3dVector(
        np.asarray(mesh1.triangle_normals)
        [:len(mesh1.triangle_normals)//2, :])
    print(mesh1.triangles)
    draw_geometries([mesh1])

    print("Painting the mesh")
    mesh1.paint_uniform_color([1, 0.706, 0])
    draw_geometries([mesh1])
