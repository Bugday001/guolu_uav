# px4_offboard

`roslaunch px4_offboard XLOAM_fast_planner.launch`

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

然后就运行sh就行