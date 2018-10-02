# -*- coding: utf-8 -*-
from open3d import *
import matplotlib.pyplot as plt


if __name__ == "__main__":
    print("Read Redwood dataset")
    color_raw = read_image("../../TestData/RGBD/color/00000.jpg")
    depth_raw = read_image("../../TestData/RGBD/depth/00000.png")
    # RGBD画像を生成する.
    rgbd_image = create_rgbd_image_from_color_and_depth(
        color_raw, depth_raw);
    # Image of size 640x480, with 3 channels.
    print(color_raw)
    # Color image : 640x480, with 1 channels.
    # Depth image : 640x480, with 1 channels.
    print(rgbd_image)
    plt.subplot(1, 2, 1)
    plt.title('Redwood grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('Redwood depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()
    pcd = create_point_cloud_from_rgbd_image(rgbd_image, PinholeCameraIntrinsic(
            PinholeCameraIntrinsicParameters.PrimeSenseDefault))
#                    PinholeCameraIntrinsic.prime_sense_default)
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    draw_geometries([pcd])

