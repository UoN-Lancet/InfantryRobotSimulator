# InfantryRobotSimulator
InfantryRobotSimulator
## 安装过程
创建catkin_ws/src
克隆仓库
catkin_make

## 依赖
```bash
sudo apt-get install ros-$(distro)-joint-state* 
sudo apt-get install ros-$(distro)-robot-state* 
sudo apt-get install ros-$(distro)-ros-gazebo*
```

## 运行
roslaunch infantry_description gazebo.launch #打开gazebo模拟器，显示机器人
roslaunch infantry_description controller.launch #打开控制器，加载控制器

## 控制
rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 1.0" # Rev35是中层对地盘的旋转轴，其他轴对应关系在xarco中可以查到

## TF
见Doc/frames.pdf

## Todo
控制逻辑，使用python或者cpp实现完整控制逻辑

