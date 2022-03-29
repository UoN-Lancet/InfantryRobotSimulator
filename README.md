# InfantryRobotSimulator


## 本仓库属于[诺丁汉大学RoboMaster团队](https://github.com/UoN-Lancet) AnywareInLoop 系统

![avatar](./doc/1.png)
## 安装过程
创建catkin_ws/src
克隆仓库
catkin_make

## 依赖
```bash
sudo apt-get install ros-$(distro)-joint-state* 
sudo apt-get install ros-$(distro)-robot-state* 
sudo apt-get install ros-$(distro)-ros-gazebo*
sudo apt-get install ros-$(distro)-ros-controll*
```

## 运行
```bash
roslaunch infantry_description gazebo.launch # 打开gazebo模拟器，显示机器人
```
```bash
roslaunch infantry_description controller.launch # 打开控制器，加载控制器
```
```bash
rosrun infantry_motion_control chassis # 打开底盘控制节点
```
```bash
rosrun infantry_motion_control middlePart # 打开中层控制节点
```
## 控制
```bash
rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 1.0" # Rev35是中层对地盘的旋转轴，其他轴对应关系在xarco中可以查到
```

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 10, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' # 朝云台指向前进
```

## TF
见Doc/frames.pdf

## Todo
<del>控制逻辑，使用python或者cpp实现完整控制逻辑</del>

加装传感器，实现SLAM并发布odom

