# 大型火电锅炉封闭炉膛水冷壁无人机自动巡检系统
**系统介绍：**
  机载Linux电脑使用UART与飞控进行通信，使用I2C和SPI总线与陀螺仪和气压计通信，控制Linux开发板或飞控IO输出PWM频率与占空比，适配灯光、脚架舵机和相机等设备。基于ROS框架，改良DLIO算法进行三维激光SLAM，建立锅炉炉膛模型并实现炉内精确定位。修改路径规划算法，根据定位位姿与从总控平台接受的目标点进行轨迹规划，编写自主飞行程序，串联途径点并实时更新状态，实现无人机自主飞行与返航。飞行过程可在Rviz中可视化，使用PCL库对周围点云进行滤波裁剪，更新路径规划状态机实现实时避障。

本工程为软件主题，需要配合[Fast-Planner-modified](https://github.com/Bugday001/Fast-Planner-modified)和[DLIO](https://github.com/Bugday001/direct_lidar_inertial_odometry)一起使用。

## Run

启动各部分程序：

`roslaunch px4_offboard XLOAM_fast_planner.launch`

保存设置基准地图：

`rosservice call /robot/dlio_map/save_pcd "leaf_size: 0.1 
save_path: '/home/denext/catkin_ws/src/px4_offboard/models'"`

用meshLab得到变换矩阵，用鼠标左键单击File下的Save project as

把里面的变换矩阵复制到config里的transform.yaml里

得到变换矩阵和原始点云放在model下。修改transform.yaml，配置点云路径和变换矩阵。


`transformation.launch`
运行这个之后得到变换后的点云

下面两个一起开
`roslaunch px4_offboard XLOAM_fast_planner.launch`

`matchHome.launch`

先点3d viewer的小红叉，然后再ctrl+c关闭

匹配参数就保存到了params里

~~然后就运行sh就行~~