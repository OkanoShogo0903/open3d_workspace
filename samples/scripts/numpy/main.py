# -*- coding: utf-8 -*-
from open3d import *
import numpy as np
import sys, copy

if __name__ == "__main__":
    '''
        Open3Dのデータ構造は、numpyと互換性があるよ.
    '''

    # generate some neat n times 3 matrix using a variant of sync function
    x = np.linspace(-3, 3, 401)
    mesh_x, mesh_y = np.meshgrid(x,x)
    z = np.sinc((np.power(mesh_x,2)+np.power(mesh_y,2)))
    xyz = np.zeros((np.size(mesh_x),3))
    xyz[:,0] = np.reshape(mesh_x,-1)
    xyz[:,1] = np.reshape(mesh_y,-1)
    xyz[:,2] = np.reshape(z,-1)
    print('xyz')
    print(xyz)
    # z = sin(x^2 + y^2)/(x^2 + y^2)

    # Numpy ------> Open3D
    # 
    # Vector3dVector -> PointCloud -> *.ply
    # 
    # Pass xyz to Open3D.PointCloud and visualize
    pcd = PointCloud()
    pcd.points = Vector3dVector(xyz)
    write_point_cloud("../../TestData/sync.ply", pcd)

    # Open3D ------> Numpy
    # 
    # *.ply -> PointCloud -> numpyの配列
    # 
    # Load saved point cloud and transform it into NumPy array
    pcd_load = read_point_cloud("../../TestData/sync.ply")
    xyz_load = np.asarray(pcd_load.points)
    print('xyz_load')
    print(xyz_load)

    # visualization
    draw_geometries([pcd_load])
