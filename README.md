# 点云建图

该ros包需要配合ORB-SLAM2进行使用，由ORB-SLAM2或者其他的位姿估计节点输出相机的位姿，该节点接受彩色图与深度图生成点云，根据位姿估计器输出的相机位姿Tcw对局部点云进行拼接。

详细的教程请参考csdn博客系列教程：[ORB-SLAM2 在线构建稠密点云](https://blog.csdn.net/crp997576280/article/details/88899163)

使用说明

```
git clone https://github.com/RuPingCen/pointcloud_mapping.git 　

roslaunch pointcloud_mapping pointcloud_mapping.launch
```


