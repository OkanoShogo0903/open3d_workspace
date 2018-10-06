#!/usr/bin/python
# -*- coding: utf-8 -*-

# [Import]------------------------------->
import sys
import copy
import threading
import types

import numpy as np
from open3d import *

import rospkg
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#from std_msgs.msg import String
# [ImportScripts]------------------------------->
import util

# sample -------------------------------------->>>

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    draw_geometries([source_temp, target])


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([1, 0, 0])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])


def colorIcp(source, target):
    '''
        位置合わせにジオメトリと色情報を用いるICPを使っている.
        色情報は平面上の位置合わせで使われているらしい.
    '''
    try:
        print("1. Load two point clouds and show initial pose")
        #source = read_point_cloud("../samples/TestData/ColoredICP/frag_115.ply")
        #target = read_point_cloud("../samples/TestData/ColoredICP/frag_116.ply")

        # estimate
        estimate_normals(source, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(target, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))

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
    except KeyboardInterrupt:
        sys.exit()


# [ClassDefine]-------------------------->
class PointCloud():
    def __init__(self):
        # get the file path.
        self.package_path       = rospkg.RosPack().get_path('open3d_workspace')

        # ROS Subscriber ----->>>
        self.realsense_sub      = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback)

        # ROS Publisher ------>>>
        #self.robot_arm_pub      = rospy.Publisher('/move/servo_url', String, queue_size=1)

        # Parameter set ------>>>
        # Set rospy to execute a shutdown function when exiting --->>>
        # rospy.on_shutdown(self.shutdown)

        self.lock = threading.Lock()
 
# [CallBack]---------------------------------->
    def callback(self, data):
        if self.lock.acquire(False): # <--- ブロッキングしないようにする
            input_pcl = util.convert_pcl(data)
            print
            #print input_pcl.points[2]
            #np.asarray(input_pcl.points)
            #print np.asarray(input_pcl.points)[:][:]

            #input_pcl = select_down_sample(input_pcl, )

            # カメラから1.5[m]以上の距離のものを弾く
            #new_pcd = PointCloud()
            #pcd_tree = KDTreeFlann(input_pcl)
            #[k, idx, _] = pcd_tree.search_radius_vector_3d([0,0,0], 1.5)
            # 確認用にGreenに塗る
            #np.asarray(input_pcl.colors)[idx[1:], :] = [0, 1, 0]
            #np.asarray(input_pcl.points)[:, :] = [0, 1, 0]
            #np.asarray(input_pcl.points)[idx[1:], :] = [0, 1, 0]
            
            #draw_geometries([new_pcd])
            print

            #print(np.asarray(input_pcl.points))
            print input_pcl

            #colorIcp(
            self.calcPointCloud(
                    source = input_pcl,
                    #target = read_point_cloud(self.package_path + "/etc/sandwich001.ply"),
                    target = read_point_cloud(self.package_path + "/etc/sandwich002.ply")
                    #target = read_point_cloud("../samples/TestData/ICP/cloud_bin_0.pcd"),
                    #target = input_pcl
                    )

            # 点群データを編集したい時使うやつ.
            #draw_geometries_with_editing([input_pcl])

            self.lock.release() # <--- 忘れずに解放する


# []---------------------------------->
    def calcPointCloud(self, source, target):
        # ポイントクラウドデータを読み込み、可視化する.
        #source = read_point_cloud("../samples/TestData/ICP/cloud_bin_0.pcd")
        #target = read_point_cloud("../samples/TestData/ICP/cloud_bin_1.pcd")
        #target = input_pcl

        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)
        #draw_registration_result(source, target, np.identity(4))

        # グローバル位置合わせでは大幅にダウンサンプリングを行う.
        source = uniform_down_sample(source, 10)
        target = uniform_down_sample(target, 10)
        source_down = voxel_down_sample(source, 0.01)
        target_down = voxel_down_sample(target, 0.01)

        # 点法線推定を行う.
        estimate_normals(source_down, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(source, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(target, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))

        #print np.asarray(target_down.normals)[:10,:]
        #print np.asarray(source_down.normals)[:10,:]

        # 各点における、FPFH特徴量を計算する.
        print("1. Compute FPFH feature with search radius 0.25")
        source_fpfh = compute_fpfh_feature(source_down,
                KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
        target_fpfh = compute_fpfh_feature(target_down,
                KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))

        # ターゲットのポイントクラウド内における対応点.
        print("2. RANSAC registration on downsampled point clouds.")
        result_ransac = registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh, 0.075,
                TransformationEstimationPointToPoint(False), 4,
                [CorrespondenceCheckerBasedOnEdgeLength(0.9),
                CorrespondenceCheckerBasedOnDistance(0.075)],
                RANSACConvergenceCriteria(4000000, 500))
        print(result_ransac)
        #draw_registration_result(source_down, target_down,
        #        result_ransac.transformation)

        # グローバル位置合わせはパフォーマンスの問題で大幅にダウンサンプリング
        # されたデータが用いられるため、精度はあまりタイトなものではない.
        # そのため、ローカル位置合わせにPointToPlaneICPを用いる.
        print("3. Point-to-plane ICP registration is applied on original point")
        result_icp = registration_icp(source, target, 0.02,
                result_ransac.transformation,
                TransformationEstimationPointToPlane())
                #TransformationEstimationPointToPoint())
        print(result_icp)
        draw_registration_result(source, target, result_icp.transformation)


if __name__ == "__main__":
    rospy.init_node('open3d_workspace')

    point_cloud = PointCloud()
    
    #rate = rospy.Rate(10.0)

    rospy.spin()
