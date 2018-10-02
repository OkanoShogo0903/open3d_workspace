# -*- coding: utf-8 -*-
# rgbd_redwood.py
#conda install pillow matplotlib
import types
#from py3d import *
from open3d import *
#from py3d import *
import matplotlib.pyplot as plt

def run():
        print("Read Redwood dataset")
        color_raw = read_image("./TestData/RGBD/color/00000.jpg")
        depth_raw = read_image("./TestData/RGBD/depth/00000.png")
        rgbd_image = create_rgbd_image_from_color_and_depth(
                color_raw, depth_raw);
        print color_raw
        print depth_raw
        print "*"*30
        print type( rgbd_image.color )
        print "*"*30

        plt.figure(figsize=(12,5))    
        # subplot(行,列,何番目のプロットか) 複数のグラフを整列させて描写したい時につかう
        plt.subplot(3, 3, 1)
        plt.title('Redwood original image')
        plt.imshow(color_raw) # rgbの3チャンネルの情報を持つ640x480の行

        plt.subplot(3, 3, 3)
        plt.title('Redwood grayscale image')
        plt.imshow(rgbd_image.color,cmap='gray') # 深度の1チャンネルの情報を持つ640x480の行

        plt.subplot(3, 3, 7)
        plt.title('Redwood depth image')
        plt.imshow(rgbd_image.depth)

        #plt.show()
        #try:    # py3d old version?
        #pcd = create_point_cloud_from_rgbd_image(rgbd_image,\
        #        PinholeCameraIntrinsic.prime_sense_default)
        #except:    # new version?
        pcd = create_point_cloud_from_rgbd_image(rgbd_image,\
                PinholeCameraIntrinsic.get_prime_sense_default())        
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        draw_geometries([pcd])

if __name__ == "__main__":
    run()
