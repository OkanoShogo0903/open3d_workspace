## HowToUse
```
$ roslaunch realsense2_camera rs_rgbd.launch
$ rosrun rqt_reconfigure rqt_reconfigure
$ roslaunch open3d_workspace open3d_workspace.launch
```

## Install
- Open3d install  
```
$ python -m pip install --user open3d-python
```
- install realsense driver
$ git clone https://github.com/intel-ros/realsense.git
## MEMO
```
1. Compute FPFH feature with search radius 0.25
2. RANSAC registration on downsampled point clouds.
Floating point exception (core dumped)
```

- realsenseを起動する.
```
$ roslaunch realsense2_camera rs_rgbd.launch
```

- realsenseの設定ファイル
realsenseのlaunch起動中に以下のコードでGUIから操作できる.
変更はリアルタイムで反映される.
```
$ rosrun rqt_reconfigure rqt_reconfigure
```
