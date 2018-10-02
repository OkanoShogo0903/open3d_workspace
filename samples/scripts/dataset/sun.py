# -*- coding: utf-8 -*-
from open3d import *
import matplotlib.pyplot as plt

if __name__ == "__main__":
        print("Read SUN dataset")
        color_raw = read_image("../../TestData/RGBD/other_formats/SUN_color.jpg")
        depth_raw = read_image("../../TestData/RGBD/other_formats/SUN_depth.png")
        # ここが若干違う.
        # ---> create_rgbd_image_from_color_and_depth
        rgbd_image = create_rgbd_image_from_sun_format(color_raw, depth_raw);
        print(rgbd_image)
        plt.figure(figsize=(12,5))    
        plt.subplot(1, 3, 1)
        plt.title('SUN original image')
        plt.imshow(color_raw)
        plt.subplot(1, 3,  2)
        plt.title('SUN grayscale image')
        plt.imshow(rgbd_image.color, cmap='gray')
        plt.subplot(1, 3, 3)
        plt.title('SUN depth image')
        plt.imshow(rgbd_image.depth)
        plt.show()
        pcd = create_point_cloud_from_rgbd_image(rgbd_image, PinholeCameraIntrinsic(
                PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        #pcd = create_point_cloud_from_rgbd_image(rgbd_image, PinholeCameraIntrinsic(
        #                PinholeCameraIntrinsic.PrimeSenseDefault))
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        draw_geometries([pcd])
