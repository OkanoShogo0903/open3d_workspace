# -*- coding: utf-8 -*-
from open3d import *
import copy
import numpy as np

def draw_registration_result(source, target, transformation):
    # 元のデータを変換するのでdeepcopyする.
    # <deepcopy>とは??? ---> id(ポインタ)も参照するものも異なるオブジェクトを生成する.
    #source_temp = copy.deepcopy(source)
    #target_temp = copy.deepcopy(target)
    source_temp = copy.copy(source)
    target_temp = copy.copy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    # アライメント変換を行う.
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])


if __name__ == "__main__":
    # とりあえずポイントクラウドを読み込む.
    source = read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    target = read_point_cloud("../../TestData/ICP/cloud_bin_1.pcd")
    threshold = 0.02
    trans_init = np.asarray(
                [[0.862, 0.011, -0.507,  0.5],
                [-0.139, 0.967, -0.215,  0.7],
                [0.487, 0.255,  0.835, -1.4],
                [0.0, 0.0, 0.0, 1.0]])
    # ソースとターゲットを描写.
    draw_registration_result(source, target, trans_init)

    print("Initial alignment")
    # 重要なメトリクスを計算
    # evaluation : 評価
    evaluation = evaluate_registration(source, target,
            threshold, trans_init)
    print(evaluation)
    # RegistrationResult with fitness = 0.174723, inlier_rmse = 0.011771,
    #           and correspondence_set size of 34741
    # Access transformation to get result.

    # POINT-TO-POINT (p2p) ICP ------>>>
    print("Apply point-to-point ICP")
    reg_p2p = registration_icp(source, target, threshold, trans_init,
            TransformationEstimationPointToPoint()) # <--- ポイントツーポイントICP対象の残差行列とヤコビ行列を計算する関数を提供
    print(reg_p2p)
    # RegistrationResult with fitness = 0.372450, inlier_rmse = 0.007760, and correspondence_set size of 74056
    print("Transformation is:")
    print(reg_p2p.transformation)
     
    # Transformation is:
    # [[ 0.83924644  0.01006041 -0.54390867  0.64639961]
    #  [-0.15102344  0.96521988 -0.21491604  0.75166079]
    #  [ 0.52191123  0.2616952   0.81146378 -1.50303533]
    #  [ 0.          0.          0.          1.        ]]
    print("")
    draw_registration_result(source, target, reg_p2p.transformation)

    # POINT-TO-PLANE ICP ------>>>
    # pointo-to-pointと比較して、以下の点が異なる.
    #   - 目的関数に点の法線を含む.
    #   - 収束速度が比較的早い.
    # また、仕様として今回はデータセットに点の法線を含むが、法線が与えられていたいときは頂点の法線推定を行ってから計算する.
    print("Apply point-to-plane ICP")
    reg_p2l = registration_icp(source, target, threshold, trans_init,
            TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    # Transformation is:
    # [[ 0.84023324  0.00618369 -0.54244126  0.64720943]
    #  [-0.14752342  0.96523919 -0.21724508  0.81018928]
    #  [ 0.52132423  0.26174429  0.81182576 -1.48366001]
    #  [ 0.          0.          0.          1.        ]]
    print("")
    draw_registration_result(source, target, reg_p2l.transformation)
