# 点云建图

该ros包需要配合ORB-SLAM2进行使用，由ORB-SLAM2或者其他的位姿估计节点输出相机的位姿，该节点接受彩色图与深度图生成点云，根据位姿估计器输出的相机位姿Tcw对局部点云进行拼接。

详细的教程请参考csdn博客系列教程：[ORB-SLAM2 在线构建稠密点云](https://blog.csdn.net/crp997576280/article/details/88899163)

![Distribute SLAM](https://github.com/RuPingCen/pointcloud_mapping/raw/v1.0.0/images/fig1.png)

使用说明

```
git clone https://github.com/RuPingCen/pointcloud_mapping.git 　

roslaunch pointcloud_mapping pointcloud_mapping.launch
```
# Useage of Distribute SLAM

1. clone the pose estimater node

```
cd catkin_ws/src

git clone https://github.com/RuPingCen/pointcloud_mapping.git 

cd ORB_SLAM2

./build.sh

./build_ros.sh

roslaunch pointcloud_mapping tum1.launch
```

2. clone the mapping node 

```
cd catkin_ws/src

git clone https://github.com/RuPingCen/pointcloud_mapping.git 

cd ..

catkin_make

roslaunch pointcloud_mapping tum1.launch
```

## RGBD Model

1. download the TUM rosbag form the [website](https://blog.csdn.net/crp997576280/article/details/88899163), then open a terminal and play the rosbag

```
rosbag play rgbd_dataset_freiburg1_room.bag
```

2. create the catkin workspace and launch the mapping node with launch file

```
cd catkin_ws/src

git clone https://github.com/RuPingCen/pointcloud_mapping.git 

cd ..

catkin_make

roslaunch pointcloud_mapping tum1.launch
```

3. 


## STEREO Model
```
roslaunch pointcloud_mapping pointcloud_mapping.launch
```





