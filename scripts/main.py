#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#@article{Zhou2018,
#       author    = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
#       title     = {{Open3D}: {A} Modern Library for {3D} Data Processing},
#       journal   = {arXiv:1801.09847},
#       year      = {2018},
#}
# 
#@Supported compilers
# GCC 4.8 and later on Linux
#
# [Import]------------------------------->
import sys
import time
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
import util # realsense topic -> open3d data

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
        self.voxel_size = 0.01

        #self.lock = threading.Lock()
        self.lock = threading.Lock()
        self.num  = 0


# [CallBack]---------------------------------->
    def callback(self, data):
        self.num = self.num + 1
        num = self.num
        if self.lock.acquire(0): # <--- ノンブロッキングでmutex処理
            input_pcl = util.convert_pcl(data)
            #input_pcl = read_point_cloud(self.package_path + "/etc/dataset001.ply")

            #draw_geometries([input_pcl])

            # 点群データを編集したい時使うやつ.
            #draw_geometries_with_editing([input_pcl])
            #draw_geometries_with_editing([read_point_cloud(self.package_path + "/etc/sandwich003.ply")])

            # 点群データの保存
            #print write_point_cloud(self.package_path + "/etc/dataset001.ply", input_pcl, write_ascii=False, compressed=False)
            print
            print num
            print input_pcl
            #print np.asarray(input_pcl.points)[:][:]
            #print(np.asarray(input_pcl.points))

            # カメラから一定範囲以上の距離の点群を取り除く
            VALID_DISTANCE = 1.15 # [m]
            pcd_tree = KDTreeFlann(input_pcl)
            [k, idx, _] = pcd_tree.search_radius_vector_3d([0,0,0], VALID_DISTANCE)
            input_pcl = select_down_sample(input_pcl, idx)
            print input_pcl

            #np.set_printoptions(threshold=100)

            # 色で閾値外の値を取り除く.
            rgb_bottom, rgb_top = [0, 0, 0], [1, 0.25, 1]
            a = np.asarray(input_pcl.colors)
            tf_array = np.any((a <= rgb_top) & (a >= rgb_bottom) == False, axis=1)

            # get tuple **array** index.
            over_idx, under_idx = np.where(tf_array == True), np.where(tf_array == False)
            # change tuple to list.
            over_idx, under_idx = list(over_idx[0]), list(under_idx[0])

            #try:
            input_pcl = select_down_sample(input_pcl, over_idx)
            #execute TypeError:
            #    pass

            print input_pcl

            # 確認用にGreenに塗る
            #np.asarray(new_pcd.colors)[idx[1:], :] = [0, 1, 0]
            #np.asarray(input_pcl.colors)[idx[1:], :] = [0, 1, 0]
            #np.asarray(input_pcl.points)[:, :] = [0, 1, 0]
            #np.asarray(input_pcl.points)[idx[1:], :] = [0, 1, 0]

            #draw_geometries([input_pcl])

            #colorIcp(
            self.calcPointCloud(
                    source = input_pcl,
                    #target = read_point_cloud(self.package_path + "/etc/sandwich001.ply"),
                    target = read_point_cloud(self.package_path + "/etc/sandwich002.ply")
                    #target = read_point_cloud("../samples/TestData/ICP/cloud_bin_0.pcd"),
                    )
            self.calcPointCloud(
                    source = read_point_cloud(self.package_path + "/etc/sandwich002.ply"),
                    #target = read_point_cloud(self.package_path + "/etc/sandwich001.ply"),
                    target = input_pcl
                    #target = read_point_cloud("../samples/TestData/ICP/cloud_bin_0.pcd"),
                    )

            self.lock.release() # <--- 忘れずに解放する


# []---------------------------------->
    def calcPointCloud(self, source, target):
        # 処理前の時刻
        pre_time = time.time() 
 
        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)
        #draw_registration_result(source, target, np.identity(4))

        # グローバル位置合わせでは大幅にダウンサンプリングを行う.
        #source = uniform_down_sample(source, 3)
        #target = uniform_down_sample(target, 3)
        source_down = voxel_down_sample(source, self.voxel_size)
        target_down = voxel_down_sample(target, self.voxel_size)

        # 点法線推定を行う.
        print("1. Estimate feature")
        start = time.time()
        estimate_normals(source_down, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(source, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        estimate_normals(target, KDTreeSearchParamHybrid(
                radius = 0.1, max_nn = 30))
        print("Estimate feature took %.3f [s]\n" % (time.time() - start))

        #print np.asarray(target_down.normals)[:10,:]
        #print np.asarray(source_down.normals)[:10,:]

        # 各点における、FPFH特徴量を計算する.
        print("2. Compute FPFH feature with search radius")
        start = time.time()
        source_fpfh = compute_fpfh_feature(source_down,
                KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
        target_fpfh = compute_fpfh_feature(target_down,
                KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
        print("calc FPFH feature took %.3f [s]\n" % (time.time() - start))

        # ターゲットのポイントクラウド内における対応点.
        print("3-a. RANSAC registration")
        start = time.time()
        distance_threshold_ransac = self.voxel_size * 1.5
        result_ransac = registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh, distance_threshold_ransac,
                TransformationEstimationPointToPoint(False), 4,
                [CorrespondenceCheckerBasedOnEdgeLength(0.9),
                CorrespondenceCheckerBasedOnDistance(0.075)],
                RANSACConvergenceCriteria(4000000, 500))
        print(result_ransac)
        print("Global registration took %.3f [s]\n" % (time.time() - start))

        print("3-b. FAST registration")
        start = time.time()
        distance_threshold_fast = self.voxel_size * 0.5
        result_fast = registration_fast_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh,
                FastGlobalRegistrationOption(
                maximum_correspondence_distance = distance_threshold_fast))
        print(result_fast)
        print("Fast registration took %.3f [s]\n" % (time.time() - start))

        #draw_registration_result(source_down, target_down,
        #        result_ransac.transformation)

        result_global = max(result_ransac, result_fast)

        print("4. Point-to-plane ICP registration")
        start = time.time()
        result_icp = registration_icp(source, target, 0.02,
                result_ransac.transformation,
                TransformationEstimationPointToPlane())
                #TransformationEstimationPointToPoint())
        print(result_icp)
        print("ICP registration took %.3f [s]\n" % (time.time() - start))

        # 経過時間を表示
        print("calc time : %.3f + [s]" % (time.time() - pre_time))

        draw_registration_result(source, target, result_icp.transformation)


if __name__ == "__main__":
    rospy.init_node('open3d_workspace')

    point_cloud = PointCloud()
    
    #rate = rospy.Rate(10.0)

    rospy.spin()
