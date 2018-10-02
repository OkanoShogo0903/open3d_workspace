# -*- coding: utf-8 -*-
from open3d import *
import numpy as np

if __name__ == "__main__":

    print("Testing kdtree in py3d ...")
    print("Load a point cloud and paint it gray.")
    # 最近傍クエリーの前処理ステップ.
    # 点群を読み取り、KDTreeを構築する.
    pcd = read_point_cloud("../../TestData/Feature/cloud_bin_0.pcd")
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_tree = KDTreeFlann(pcd)

    # 1500番目のポイントを赤く塗りつぶす.
    print("Paint the 1500th point red.")
    pcd.colors[1500] = [1, 0, 0]

    # knn k個のnearest neighbors
    print("Find its 200 nearest neighbors, paint blue.")
    print pcd.points[1500]
        # [ 2.18009663  0.73027939 -1.32014894]
    print pcd.colors
        # std::vector<Eigen::Vector3d> with 3903 elements.
        # Use numpy.asarray() to access data.
    print pcd.colors[1500]
        # [ 1.  0.  0.]
    print pcd.colors[1500][0] # 1
    print pcd.colors[1500][1] # 0
    print pcd.colors[1500][2] # 0
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
    # ポイントカラーのvectorをnumpy配列に変換して、ポイントカラーの要素に色情報[0,0,1]をブロードキャストする.
    # idx[1]は1500番のポイントなので省く
    np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

    # rnn radius、一定距離内の点を検索する.
    print("Find its neighbors with distance less than 0.2, paint green.")
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

    print("Visualize the point cloud.")
    draw_geometries([pcd])
    print("")
