# FAST-LIO-LOCALIZATION

一个基于[FAST-LIO](https://github.com/hku-mars/FAST_LIO)的简单定位框架，能够在已建好的地图中实现重定位。

## 最新动态

- **2021-08-11:** 增加对**Open3D 0.7**的支持。
  
- **2021-08-09:** 迁移到**Open3D**以获得更好的性能。

## 1. 特性
- 在预先构建的点云地图中实现实时3D全局定位。
  通过融合低频全局定位（约0.5~0.2Hz）和来自FAST-LIO的高频里程计，整个系统具有较高的计算效率。

<div align="center"><img src="doc/demo.GIF" width=90% /></div>

- 消除里程计的累积误差。

<div align="center"><img src="doc/demo_accu.GIF" width=90% /></div>

- 初始定位可以通过RVIZ进行粗略人工估计提供，也可以由其他传感器/算法提供的位姿提供。

<div align="center">
<img src="doc/demo_init.GIF" width=49.6% />
<img src="doc/demo_init_2.GIF" width = 49.6% >
</div>


## 2. 环境要求
### 2.1 FAST-LIO的依赖项

技术上，如果你之前已经构建并运行过FAST-LIO，可以跳过2.1节。

这部分依赖项与FAST-LIO保持一致，请参考文档：https://github.com/hku-mars/FAST_LIO#1-prerequisites

### 2.2 定位模块的依赖项

- python 2.7

- [ros_numpy](https://github.com/eric-wieser/ros_numpy)

```shell
sudo apt install ros-$ROS_DISTRO-ros-numpy
```

- [Open3D](http://www.open3d.org/docs/0.9.0/getting_started.html)

```shell
pip install open3d==0.9
```

注意，在**Python 2.7**中直接使用pip安装**Open3D**可能会出现问题：
```shell
错误：包'pyrsistent'需要一个不同的Python版本：2.7.18不在'>=3.5'的范围内
```
你可以先安装**pyrsistent**：
```shell
pip install pyrsistent==0.15
```
然后
```shell
pip install open3d==0.9
```


## 3. 构建
克隆仓库并使用catkin_make编译：

```
    cd ~/$某个ROS工作空间目录$/src
    git clone https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION.git
    cd FAST_LIO_LOCALIZATION
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- 记得在构建前source livox_ros_driver（遵循[livox_ros_driver](https://github.com/hku-mars/FAST_LIO#13-livox_ros_driver)的说明）
- 如果你想使用自定义构建的PCL，请将以下行添加到~/.bashrc文件中
  ```export PCL_ROOT={自定义PCL路径}```


## 4. 运行定位
### 4.1 示例数据集

大型地下车库的演示rosbag数据：
[Google Drive](https://drive.google.com/file/d/15ZZAcz84mDxaWviwFPuALpkoeK-KAh-4/view?usp=sharing) | [百度网盘 (提取码: ne8d)](https://pan.baidu.com/s/1ceBiIAUqHa1vY3QjWpxwNA);

对应的地图：[Google Drive](https://drive.google.com/file/d/1X_mhPlSCNj-1erp_DStCQZfkY7l4w7j8/view?usp=sharing) | [百度网盘 (提取码: kw6f)](https://pan.baidu.com/s/1Yw4vY3kEK8x2g-AsBi6VCw)

该地图可以使用LIO-SAM或FAST-LIO-SLAM构建。

### 4.2 运行

1. 首先，请确保你使用的是**Python 2.7**环境；

2. 运行定位，这里我们以Livox AVIA为例：

```shell
roslaunch fast_lio_localization localization_avia.launch map:=/你的地图pcd文件路径/map.pcd
```

请将`/你的地图pcd文件路径/map.pcd`修改为你自己的地图点云文件路径。

等待3~5秒，直到地图点云在RVIZ中显示；

3. 如果你使用示例rosbag数据进行测试：
```shell
rosbag play localization_test_scene_1.bag
```

或者如果你在实时运行

```shell
roslaunch livox_ros_driver livox_lidar_msg.launch
```
请将**livox_lidar_rviz.launch**中的**publish_freq**设置为**10Hz**，以确保单次扫描中有足够的点用于全局定位。
对更高频率的支持即将推出。

4. 提供初始位姿
```shell
rosrun fast_lio_localization publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0 
```
数值 **14.5 -7.5 0 -0.25 0 0** 表示地图坐标系中的6D位姿 **x y z 偏航角 俯仰角 横滚角**，
这是针对 **localization_test_scene_1.bag** 的粗略初始猜测。

初始猜测也可以通过RVIZ中的'2D位姿估计'工具提供。

注意，在初始化阶段，最好让机器人保持静止。或者如果你播放数据包，先播放大约0.5秒，然后暂停播放，直到初始化成功。


## 相关工作
1. [FAST-LIO](https://github.com/hku-mars/FAST_LIO)：一个计算高效且稳健的激光雷达惯性里程计（LIO）包
2. [ikd-Tree](https://github.com/hku-mars/ikd-Tree)：一种用于3D kNN搜索的先进动态KD-Tree。
3. [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)：集成了FAST-LIO和[Scan-Context](https://github.com/irapkaist/scancontext) **闭环检测**模块的系统。
4. [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization)：一个基于LIO-SAM的、能够在已建地图上对机器人进行重定位的简单系统。


## 致谢
感谢[FAST-LIO](https://github.com/hku-mars/FAST_LIO)和[LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization)的作者们。

## 待办事项
1. 解决已发布里程计和tf的时间戳问题；
2. 使用积分后的点云进行全局定位；
3. 将全局定位与FAST-LIO的状态估计融合，平滑定位轨迹；
4. 持续更新中...
