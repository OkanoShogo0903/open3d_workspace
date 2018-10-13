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
import logging
import math

import numpy as np
from open3d import *

import rospkg
import rospy
#from geometry_msgs.msg import Transform
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#from std_msgs.msg import String
# [ImportScripts]------------------------------->
import util # realsense topic -> open3d data

# sample -------------------------------------->>>

def draw_registration_result_original_color(source, target, transformation):
    if DEBUG == True:
        source_temp = copy.deepcopy(source)
        source_temp.transform(transformation)
        draw_geometries([source_temp, target])
    return


def draw_registration_result(source, target, transformation):
    if DEBUG == True:
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        #source_temp.paint_uniform_color([1, 0.706, 0])
        #target_temp.paint_uniform_color([1, 0, 0])
        source_temp.transform(transformation)
        #camera = 
        draw_geometries([source_temp, target_temp])
    return


# [ClassDefine]------------------------->
class PointCloud():
    def __init__(self):
        # get the file path.
        self.package_path       = rospkg.RosPack().get_path('open3d_workspace')

        # ROS Subscriber ------------->>>
        self.realsense_sub      = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback, queue_size=1)

        # ROS Publisher -------------->>>
        self.object_point_sub   = rospy.Publisher('/point_cloud/object_point', Point, queue_size=1)

        # Object Parameter ----------->>>
        self.TARGET_OBJECT_RADIUS = 0.045 # [m]

        # Environment Parameter ------>>>
        self.RGB_BOTTOM, self.RGB_TOP = [0, 0, 0], [1, 0.25, 1]
        self.VALID_DISTANCE = 1.15 # [m] 有効な点群の距離.

        # Common Parameter ----------->>>
        self.VOXEL_SIZE = 0.01 # [m]
        self.MAX_POINTS = 2000 # 入力点群の最大許容数.
        self.OFFSET = [0.00, 0.00, 0.00] # [m] 点群は表面の座標を表すので.
        
        # Output setting ------------->>>
        #np.set_printoptions(threshold=100)

        # Set rospy to execute a shutdown function when exiting --->>>
        # rospy.on_shutdown(self.shutdown)

# [CallBack]---------------------------------->
    def callback(self, data):
        '''
            realsenseのパッケージから点群のtopicを受け取って、
            オブジェクトのカメラ位置をpublishする.
        '''
        # 現在時刻に近いデータなら実行.
        if rospy.Time.now().secs == data.header.stamp.secs:
            # Set pcl data. ----->>>
            input_pcl = util.convert_pcl(data)
            #input_pcl = read_point_cloud(self.package_path + "/etc/dataset001.ply")

            model = read_point_cloud(self.package_path + "/etc/sandwich002.ply")
            #model = read_point_cloud("../samples/TestData/ICP/cloud_bin_0.pcd"),
            #model = read_point_cloud(self.package_path + "/etc/sandwich001.ply"),

            # Process. ----->>>
            #draw_geometries([input_pcl])

            # 点群データを編集したい時使うやつ.
            #draw_geometries_with_editing([input_pcl])
            #draw_geometries_with_editing([read_point_cloud(self.package_path + "/etc/sandwich003.ply")])

            # 点群データの保存
            #print write_point_cloud(self.package_path + "/etc/dataset001.ply", input_pcl, write_ascii=False, compressed=False)

            print 
            print input_pcl

            print "pcdtree 0"
            # nanを含むものがあれば終了.
            #print np.any(np.isnan(np.asarray(input_pcl.points)))
            #if np.any(np.isnan(np.asarray(input_pcl.points))) == True:
            #    return 
            input_pcl = self.preprocess(input_pcl)

            # 確認用にGreenに塗る
            #np.asarray(input_pcl.colors)[idx[1:], :] = [0, 1, 0]
            #np.asarray(input_pcl.points)[:, :] = [0, 1, 0]
            #np.asarray(input_pcl.points)[idx[1:], :] = [0, 1, 0]

            #draw_geometries([input_pcl])

            if len( input_pcl.points ) > 0:
                #self.colorIcp(
                #result_a = self.calcPointCloud(
                #        source = input_pcl,
                #        target = model
                #        )
                result = self.calcPointCloud(
                        source = model,
                        target = input_pcl
                        )
                
                # 複数の認識結果から一番良い結果を使う.
                #eval_a = (result_a.fitness * result_a.inlier_rmse)
                #eval_b = (result_b.fitness * result_b.inlier_rmse)

                #if eval_a > eval_b:
                #    print "a is better *************************************************"
                #    result = result_a
                #else:
                #    print "b is better *************************************************"
                #    result = result_b

                # Publish.
                if result.inlier_rmse == 0 or result.fitness == 0:
                    logging.debug('Detect fail.')
                else:
                    logging.debug('Detect success.')
                    print result
                    # Publish model coordinate in robot system.
                    # 姿勢情報をカメラから見た現実座標に変換する.

                    if 1:
                        '''
                        # 自作手法その1 --->
                        # 点群が密であるところは重みが高く設定されると同義なので、
                        # vexelによって重み付けを消して、その点群重心から座標を求める.
                        '''
                        t_down  = voxel_down_sample(model, self.VOXEL_SIZE)
                        x, y, z = self.returnMovedCenterGravityFromTransformation(t_down, result.transformation)
                        x, y, z = self.returnObjectCenterFromSurfaceCenter(x, y, z)
                        x, y, z = self.adjustOffset(x, y, z)
                        print "x",x
                        print "y",y
                        print "z",z
                        if DEBUG == True:
                            draw_registration_result(model, input_pcl, result.transformation)

                    if 1:
                        '''
                        # 自作手法その2 --->
                        # オブジェクトの姿勢によってはmodelは完全にマッチングすることはないという前提.
                        # ならば、modelの重心座標付近にあるinput点群の点を複数とって、
                        # その重心がオブジェクトの表面重心だとする方法.
                        # これを使う場合、modelは面の点群、ICPにはColorICPを用いることが望ましいはず.
                        '''
                        try:
                            t_down         = voxel_down_sample(model, self.VOXEL_SIZE)
                            x, y, z        = self.returnMovedCenterGravityFromTransformation(t_down, result.transformation)
                            print "x",x
                            print "y",y
                            print "z",z
                            pcd_tree       = KDTreeFlann(input_pcl)
                            [k, idx, _]    = pcd_tree.search_radius_vector_3d([x,y,z], self.TARGET_OBJECT_RADIUS * 1.5)
                            sand_and_noise = select_down_sample(input_pcl, idx)
                            x, y, z        = np.average( sand_and_noise.points, axis=0) # cameraのx,y,zを出す.
                            x, y, z        = self.adjustOffset(x, y, z)
                            print "x",x
                            print "y",y
                            print "z",z

                            sand_and_noise.paint_uniform_color([1, 0, 0])
                            if DEBUG == True:
                                draw_geometries([sand_and_noise, input_pcl])
                        except:
                            # publishせずに返す
                            return
                    
                    # Publish ----->>>
                    msg = Point()
                    msg.x, msg.y, msg.z = x,y,z
                    print msg
                    self.object_point_sub.publish(msg)

                    logging.debug('Publish point data')

        return


# [Math Calc]---------------------------------->
    def returnMovedCenterGravityFromTransformation(self, pcl, trans):
        '''
            Return centery gravity coordinate list (x,y,z).

            input :
                open3d.linux.open3d.PointCloud, transformation(4x4)
            output :
                float list
        '''
        n = np.insert(np.asarray(pcl.points).T, 3, 1, axis=0) # 変換行列とかけるために行を追加する.
        # 変換行列から移動後の点ごとの座標を出す.
        a = np.dot(trans, n)
        # 点群の重心位置を出す.
        [x, y, z, _] = np.average(a, axis=1) # cameraのx,y,zを出す.
        return x, y, z


    def returnObjectCenterFromSurfaceCenter(self, x, y, z):
        '''
            カメラから物体を見た時に、
            物体表面からTARGET_OBJECT_RADIUS分だけ奥に物体の中心があると考える.
            物体中心を三角比から求める.
        '''
        # Solve x in X-Z plane.
        l  = math.sqrt(pow(x,2) + pow(z,2))
        l_ = l + self.TARGET_OBJECT_RADIUS
        x_ = (l_/l) * x

        # Solve y in Y-Z plane.
        l  = math.sqrt(pow(y,2) + pow(z,2))
        l_ = l + self.TARGET_OBJECT_RADIUS
        y_ = (l_/l) * y
        
        # Solve z in X-Z plane.
        l  = math.sqrt(pow(x,2) + pow(z,2))
        l_ = l + self.TARGET_OBJECT_RADIUS
        z_ = (l_/l) * z

        return x_, y_, z_


    def adjustOffset(self, x, y, z):
        x_ = x + self.OFFSET[0]
        y_ = y + self.OFFSET[1]
        z_ = z + self.OFFSET[2]
        return x_, y_, z_

# [Open3d]---------------------------------->
    def preprocess(self, source):
        # nanがあるインデックスを取り除く. --->
        c = np.asarray(source.colors)
        p = np.asarray(source.points)

        tf_array = np.any(np.isnan(c), axis=1)
        non_nan_idx = np.where(tf_array == False)
        source = select_down_sample(source, list(non_nan_idx[0]))
        tf_array = np.any(np.isnan(p), axis=1)
        non_nan_idx = np.where(tf_array == False)
        source = select_down_sample(source, list(non_nan_idx[0]))
        print "nan remove", source

        # カメラから一定範囲以上の距離の点群を取り除く
        pcd_tree = KDTreeFlann(source)
        print "pcdtree 1"
        [k, idx, _] = pcd_tree.search_radius_vector_3d([0,0,0], self.VALID_DISTANCE)
        source = select_down_sample(source, idx)

        # 色で閾値外の値を取り除く.
        c = np.asarray(source.colors)
        tf_array = np.any((c <= self.RGB_TOP) & (c >= self.RGB_BOTTOM) == False, axis=1)

        # get tuple array index.
        over_idx, under_idx = np.where(tf_array == True), np.where(tf_array == False)
        # change tuple to list.
        over_idx, under_idx = list(over_idx[0]), list(under_idx[0])

        try:
            source = select_down_sample(source, over_idx)
        except:
            pass

        print source

        # 点が10000を超えると点法線推定に1分近くかかるためデータ数を削減する.
        while len(source.points) > self.MAX_POINTS:
            logging.debug('TOO MANY POINTS, please adjust rgb threthold')
            source = uniform_down_sample(source, 2) # do /2
        print source
        return source


    def calcPointCloud(self, source, target):
        # 処理前の時刻
        func_start = time.time() 
 
        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)
        #draw_registration_result(source, target, np.identity(4))

        # グローバル位置合わせでは大幅にダウンサンプリングを行う.
        #source = uniform_down_sample(source, 3)
        #target = uniform_down_sample(target, 3)
        source_down = voxel_down_sample(source, self.VOXEL_SIZE)
        target_down = voxel_down_sample(target, self.VOXEL_SIZE)

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
        distance_threshold_ransac = self.VOXEL_SIZE * 1.5
        result_ransac = registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh, distance_threshold_ransac,
                TransformationEstimationPointToPoint(False), 4,
                [CorrespondenceCheckerBasedOnEdgeLength(0.9),
                CorrespondenceCheckerBasedOnDistance(0.075)],
                RANSACConvergenceCriteria(4000000, 500))
        print(result_ransac)
        print("Global registration took %.3f [s]\n" % (time.time() - start))

        #print("3-b. FAST registration")
        #start = time.time()
        #distance_threshold_fast = self.VOXEL_SIZE * 0.5
        #result_fast = registration_fast_based_on_feature_matching(
        #        source_down, target_down, source_fpfh, target_fpfh,
        #        FastGlobalRegistrationOption(
        #        maximum_correspondence_distance = distance_threshold_fast))
        #print(result_fast)
        #print("Fast registration took %.3f [s]\n" % (time.time() - start))

        print("4. Point-to-plane ICP registration")
        start = time.time()
        result_icp = registration_icp(source, target, 0.02,
                result_ransac.transformation,
                TransformationEstimationPointToPlane())
                #TransformationEstimationPointToPoint())
        print(result_icp)
        print("ICP registration took %.3f [s]\n" % (time.time() - start))

        # 経過時間を表示
        print("All of function calc time : %.3f [s]" % (time.time() - func_start))

        #draw_registration_result(source, target, result_icp.transformation)
        return result_icp


    def colorIcp(self, source, target):
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

            return result_icp
        except KeyboardInterrupt:
            sys.exit()


if __name__ == "__main__":
    rospy.init_node('open3d_workspace')

    # ログレベルを DEBUG に変更
    DEBUG = False
    if DEBUG == True:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.ERROR)

    point_cloud = PointCloud()
    
    #rate = rospy.Rate(10.0)

    rospy.spin()
