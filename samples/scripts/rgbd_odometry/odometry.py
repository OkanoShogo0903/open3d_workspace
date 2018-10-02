# -*- coding: utf-8 -*-
from open3d import *
import numpy as np

if __name__ == "__main__":
    # カメラ情報の取得 ----->>>
    #print(PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(
            "../../TestData/camera_primesense.json")
    print(pinhole_camera_intrinsic.intrinsic_matrix)
        # カメラの内部パラメータの出力
        # [[  6.94474465e-310   6.95273867e-310   3.95252517e-323]
        #  [  2.42465433e-317   6.95273867e-310   1.16871110e-316]
        #  [  1.01855798e-312   6.94474450e-310   6.94474470e-310]]

    # RGBDイメージの読み込み. ----->>>
    source_color = read_image("../../TestData/RGBD/color/00000.jpg")
    source_depth = read_image("../../TestData/RGBD/depth/00000.png")
    target_color = read_image("../../TestData/RGBD/color/00001.jpg")
    target_depth = read_image("../../TestData/RGBD/depth/00001.png")

    source_rgbd_image = create_rgbd_image_from_color_and_depth(
            source_color, source_depth)
    target_rgbd_image = create_rgbd_image_from_color_and_depth(
            target_color, target_depth)
    
    target_pcd = create_point_cloud_from_rgbd_image(
            target_rgbd_image, pinhole_camera_intrinsic)

    # ??? ----->>>
    option = OdometryOption()
    odo_init = np.identity(4)
    print(option)
        # OdometryOption class.
        # iteration_number_per_pyramid_level = [ 20, 10, 5, ]
        # max_depth_diff = 0.030000
        # min_depth = 0.000000
        # max_depth = 4.000000

    # RGBD画像のペアから距離を計算. ----->>>
    # Steinbrucker et al.(2011)
    [success_color_term, trans_color_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromColorTerm(), option)
    # Park et al.(2017)
    [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromHybridTerm(), option)

    # RGBDイメージの可視化
    if success_color_term:
        print("Using RGB-D Odometry")
        print(trans_color_term)
        source_pcd_color_term = create_point_cloud_from_rgbd_image(
                source_rgbd_image, pinhole_camera_intrinsic)
        source_pcd_color_term.transform(trans_color_term)
        draw_geometries([target_pcd, source_pcd_color_term])
    if success_hybrid_term:
        print("Using Hybrid RGB-D Odometry")
        print(trans_hybrid_term)
        source_pcd_hybrid_term = create_point_cloud_from_rgbd_image(
                source_rgbd_image, pinhole_camera_intrinsic)
        source_pcd_hybrid_term.transform(trans_hybrid_term)
        draw_geometries([target_pcd, source_pcd_hybrid_term])
