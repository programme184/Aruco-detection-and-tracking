# Aruco-detection-and-tracking
### jaka机械臂启动

```shell
cd ~/catkin_ws
source ./devel/setup.sh

roslaunch jaka_ros_driver start.launch  # 启动jaka的基本底层驱动，启动后需要等待几秒，等控制柜的灯为绿色
rosrun control_msgs jaka5_server        # 接收从moveit中发来的ros topic，并对指令进行处理
roslaunch jaka5_config demo.launch      # 启动moveit 同时打开rviz
```
## 二维码标定


```shell
roslaunch realsense2_camera rs_camera.launch     #启动realsense camera
roslaunch jaka_control calib.launch              #启动运动规划
roslaunch jaka_control aruco_maker_find.launch   #相机到达aruco码上方
