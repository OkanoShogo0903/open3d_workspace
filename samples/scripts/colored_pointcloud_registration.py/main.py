# -*- coding: utf-8 -*-
from open3d import *
import copy
import numpy as np


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    draw_geometries([source_temp, target])


if __name__ == "__main__":
    '''
        位置合わせにジオメトリと色情報を用いるICPを使っている.
        色情報は平面上の位置合わせで使われているらしい.
    '''

    print("1. Load two point clouds and show initial pose")
    source = read_point_cloud("../../TestData/ColoredICP/frag_115.ply")
    target = read_point_cloud("../../TestData/ColoredICP/frag_116.ply")

    # draw initial alignment
    current_transformation = np.identity(4) # 初期値を単位行列にする.
    print current_transformation
    # ICPを使う前の点群情報を可視化する.
    draw_registration_result_original_color(
            source, target, current_transformation)

    # point to plane ICP
    current_transformation = np.identity(4);
    print("2. Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. Distance threshold 0.02.")
    result_icp = registration_icp(source, target, 0.02,
            current_transformation, TransformationEstimationPointToPlane()) # PtoP ICPと比較する
    print(result_icp)
    draw_registration_result_original_color(
            source, target, result_icp.transformation)

    # colored pointcloud registration
     # This is implementation of following paper
     # J. Park, Q.-Y. Zhou, V. Koltun,
     # Colored Point Cloud Registration Revisited, ICCV 2017
    voxel_radius = [ 0.04, 0.02, 0.01 ];
    max_iter = [ 50, 30, 14 ];
    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
    # ダウンサンプリングする際のVoxelのサイズとイテレータ（おそらく反復回数）のパラメータが異なる
    # 場合において『色付き点群の位置合わせ』の結果を比較する.
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter,radius,scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = voxel_down_sample(source, radius)
        target_down = voxel_down_sample(target, radius)

        print("3-2. Estimate normal.")
        # 点法線推定を行う.
        estimate_normals(source_down, KDTreeSearchParamHybrid(
                radius = radius * 2, max_nn = 30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius = radius * 2, max_nn = 30))

        print("3-3. Applying colored point cloud registration")
        result_icp = registration_colored_icp(source_down, target_down,
                radius, current_transformation,
                ICPConvergenceCriteria(relative_fitness = 1e-6,
                relative_rmse = 1e-6, max_iteration = iter))
        current_transformation = result_icp.transformation
        print(result_icp)
    draw_registration_result_original_color(
            source, target, result_icp.transformation)
