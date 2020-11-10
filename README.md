# multi_pc2_comb

ROS package for combining multiple pointcloud

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/multi_pc2_comb.git
   cd ..
   catkin_make
   ```

## 参数配置
 - 修改`multi_pc2_comb/launch/multi_pc2_comb.launch`
   ```Shell
   <param name="sub_topic_name_1" value="/ls_left/lslidar_point_cloud"/>
   <param name="sub_topic_name_2" value="/ls_right/lslidar_point_cloud"/>
   <param name="pub_topic_name" value="/lslidar_combined"/>
   <param name="frame_name" value="ls_comb"/>
        
   <param name="alpha_1" value="-90"/>
   <param name="beta_1" value="-45"/>
   <param name="gamma_1" value="0"/>
   <param name="offset_x_1" value="0"/>
   <param name="offset_y_1" value="0.45"/>
   <param name="offset_z_1" value="-0.2"/>
        
   <param name="alpha_2" value="90"/>
   <param name="beta_2" value="-45"/>
   <param name="gamma_2" value="0"/>
   <param name="offset_x_2" value="0"/>
   <param name="offset_y_2" value="-0.45"/>
   <param name="offset_z_2" value="-0.2"/>
   ```
    - `sub_topic_name_1`指明订阅的激光雷达1的点云话题。
    - `sub_topic_name_2`指明订阅的激光雷达2的点云话题。
    - `pub_topic_name`指明发布的拼接后的点云话题。
    - `frame_name`指明发布的点云的`header.frame_id`。
    - 用矢量偏移`offset_x/y/z`和ZYX欧拉角`alpha/beta/gamma`分别描述两个子云坐标系与拼接后点云坐标系的相对关系。

## 运行
 - 启动`multi_pc2_comb`
   ```Shell
   roslaunch multi_pc2_comb multi_pc2_comb.launch
   ```
   
