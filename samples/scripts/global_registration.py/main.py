# -*- coding: utf-8 -*-
from open3d import *
import copy
import numpy as np

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

if __name__ == "__main__":
    '''
        このグローバル位置合わせでは初期化のためにアライメントを必要としない
    '''

    # ポイントクラウドデータを読み込み、可視化する.
    print("1. Load two point clouds and disturb initial pose.")
    source = read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    target = read_point_cloud("../../TestData/ICP/cloud_bin_1.pcd")
    trans_init = np.asarray([[0.0, 1.0, 0.0, 0.0],
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    # グローバル位置合わせでは大幅にダウンサンプリングを行う.
    print("2. Downsample with a voxel size 0.05.")
    source_down = voxel_down_sample(source, 0.05)
    target_down = voxel_down_sample(target, 0.05)

    # 点法線推定を行う.
    print("3. Estimate normal with search radius 0.1.")
    estimate_normals(source_down, KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
    estimate_normals(target_down, KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))

    # 各点における、FPFH特徴量を計算する.
    print("4. Compute FPFH feature with search radius 0.25")
    source_fpfh = compute_fpfh_feature(source_down,
            KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
    target_fpfh = compute_fpfh_feature(target_down,
            KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))

    # ターゲットのポイントクラウド内における対応点.
    print("5. RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is 0.05, we use a liberal")
    print("   distance threshold 0.075.")
    result_ransac = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, 0.075,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(0.075)],
            RANSACConvergenceCriteria(4000000*10, 500*10))
    print(result_ransac)
    draw_registration_result(source_down, target_down,
            result_ransac.transformation)

    # グローバル位置合わせはパフォーマンスの問題で大幅にダウンサンプリング
    # されたデータが用いられるため、精度はあまりタイトなものではない.
    # そのため、ローカル位置合わせにPointToPlaneICPを用いる.
    print("6. Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold 0.02.")
    result_icp = registration_icp(source, target, 0.02,
            result_ransac.transformation,
            TransformationEstimationPointToPlane())
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)

