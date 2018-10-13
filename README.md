## HowToUse
```
roslaunch realsense2_camera rs_rgbd.launch
```

## Install

## MEMO
```
1. Compute FPFH feature with search radius 0.25
2. RANSAC registration on downsampled point clouds.
Floating point exception (core dumped)
```

- realsenseの設定ファイル
realsenseのlaunch起動中に以下のコードでGUIから操作できる.
変更はリアルタイムで反映される.
```
rosrun rqt_reconfigure rqt_reconfigure
```
