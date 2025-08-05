# AgileX_limo_navigation2_sim
## 一 项目概述
本仓库针对松灵LIMO小车的差速运动模式，集成了Cartographer建图与Nav2导航框架，通过参数优化解决了建图与导航过程中的典型问题，实现了：
- 长廊等复杂环境下的稳定建图（支持50米长廊场景，适配8米测距激光雷达）
- 无障碍物时的近似直线路径规划，及障碍物环境下的最优绕行（保持全局轨迹方向一致性）
- 绕障失败时的智能等待机制（障碍物清除后自动恢复规划）

## 二 环境依赖
- 操作系统：Ubuntu 20.04 LTS
- ROS 2版本：Humble 
- 核心依赖包：
  - `cartographer_ros`（建图）
  - `nav2_bringup` 及相关导航栈组件
  - `limo_description`（LIMO小车URDF描述）

## 三 功能说明

### 1. 建图模块（Cartographer）
基于Cartographer算法实现激光雷达 slam，针对以下问题进行了优化：
- **TF树结构混乱**：通过校准传感器坐标变换关系，统一坐标系（map -> odom -> base_link）
- **建图重影**：调试src/fishbot_cartographer/config/limo_lds_2d.lua参数，提升地图亚像素精度
- **长廊地图不更新**：结合Odom里程计辅助定位，限制遥控速度，延长前端匹配窗口

### 2. 导航模块（Nav2）
基于Nav2框架实现自主导航，主要特点包括：
- **全局规划器**：NavfnPlanner规划器，平衡路径直线性与绕障安全性
- **局部规划器**：调节`RegulatedPurePursuitController`参数，优化跟踪平滑性（如`smoothing_factor`、`max_deceleration`）
- **恢复行为**：配置行为树实现绕障失败时的原地等待（`Wait`节点），检测到障碍物清除后自动重新规划
- **代价地图**：优化`inflation_radius`与`cost_scaling_factor`，适配LIMO小车尺寸（差速模式轮距）

## 四 快速启动

### 1. 建立工作空间与编译ros2工程
```bash
mkdir robot_ws
cd robot_ws
git clone git@github.com:Geo-JTao/AgileX_limo_navigation2_sim.git
colcon build
```
### 2. 建图流程
```bash
########## 终端1 启动仿真环境 ##########
# 进入工作空间，刷新环境变量
cd robot_ws/
source ./install/setup.bash 
# 启动gazebo（松灵小车纹理细节丰富，没显卡加载较慢，需要等待）
ros2 launch limo_description gazebo.launch.py 

########## 终端2 启动cartographer算法建图 ##########
# 进入工作空间，刷新环境变量
cd robot_ws/
source ./install/setup.bash 
# 启动建图，此时可在rviz中看到局部灰色地图
ros2 launch fishbot_cartographer limo_cartographer.launch.py 

########## 终端3 按键控制小车移动扫描场景建立完整地图 ##########
# 注意控制小车移动速度，避免撞墙
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
# 遥控建图完成后ctrl C关闭键盘控制，并保存地图到指定路径
cd /home/robot/robot_ws/src/fishbot_navigation2/maps
ros2 run nav2_map_server map_saver_cli -t map -f your_map_name
```
### 3. 导航流程
```bash
########## 终端1 启动仿真环境 ##########
# 进入工作空间，刷新环境变量
cd robot_ws/
source ./install/setup.bash 
# 启动gazebo（松灵小车纹理细节丰富，没显卡加载较慢，需要等待）
ros2 launch limo_description gazebo.launch.py

########## 终端2 启动nav2导航 ##########
# 进入工作空间，刷新环境变量
cd robot_ws/
source ./install/setup.bash 
# 启动导航，此时可在rviz中设置导航目标点，小车规划路线开始移动
ros2 launch fishbot_navigation2 limo_navigation2.launch.py

########## 终端3（可选） 代码控制小车按写入的路径点导航 ##########
cd robot_ws/
python3 ./src/fishbot_navigation2/scripts/auto_navigator.py
```
### 4. 脚本一键启动
```bash
# 进入工作空间，刷新环境变量
cd robot_ws/
source ./install/setup.bash
sudo chmod +x ./*.sh
########## 终端1 启动仿真环境 ##########
 ./limo_start_sim.sh 
########## 终端2 启动nav2导航 ##########
./limo_navigation2_sim.sh 
########## 终端3 快速退出仿真 ##########
./kill.sh gazebo
./kill.sh nav
```
[参考资料](https://fishros.com/d2lros2/#/)
[飞书文档](https://lcnykm9wb18t.feishu.cn/wiki/FuDxwAjSAiZAGxkj05ecrAaLn7b)
