# -*- coding: utf-8 -*-
import numpy as np
#from py3d import *
from open3d import *

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    # pcb,plymxyzなどの拡張子で保存されているポイントクラウドデータをデコードする.
    pcd = read_point_cloud("../TestData/fragment.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    # ポイントクラウドを可視化する.
    #draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    # voxelはボックス-セルの略.
    # スカラー値、ベクトル値を持つ小さな立方体
    downpcd = voxel_down_sample(pcd, voxel_size = 0.05) # voxel_size[m]
    #draw_geometries([downpcd])

    print("Recompute the normal of the downsampled point cloud")
    # 点法線推定を行う.
    estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
    draw_geometries([downpcd])
    print("")

    print("Load a polygon volume and use it to crop the original point cloud")
    # 領域を選択して切り取る
    vol = read_selection_polygon_volume("../TestData/Crop/cropped.json")
    chair = vol.crop_point_cloud(pcd)
    draw_geometries([chair])
    print("")

    print("Paint chair")
    chair.paint_uniform_color([1, 0.706, 0]) # rgb空間で[0,1]の範囲で色を設定できる.
    draw_geometries([chair])
    print("")
