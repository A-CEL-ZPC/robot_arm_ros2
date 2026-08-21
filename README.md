# MyRobot

基于 **ROS 2 Humble + MoveIt 2 + ros2_control + Gazebo Classic** 的六自由度机械臂（6 轴 + 二指夹爪）运动控制工程，支持虚拟仿真与物理仿真双模式，内置完整 pick-and-place 抓取演示。

## 特性

- **双模式仿真**：`mock`（虚拟硬件，秒级启动，适合接口联调）与 `gazebo`（重力/碰撞/桌面/目标方块，完整物理验证）
- **统一模型源**：单一 URDF/Xacro，`sim_mode` 参数一键切换硬件插件；关节初值由 YAML 驱动
- **MoveIt 2 集成**：OMPL/CHOMP/Pilz 规划器、KDL 逆解、笛卡尔路径、Planning Scene 避障（桌面与目标方块自动同步）
- **完整抓取演示**：`grasp_demo` 状态机实现 接近 → 张开 → 下降 → 轻夹 → 抬升 → 搬运 → 放置 → 释放，Gazebo 中通过抓取附着机制稳定完成物理 pick-and-place
- **三话题控制面**：关节角 / 末端位姿（含笛卡尔直线）/ 夹爪开合，另支持直接调用控制器 Action
- **离线自包含**：Gazebo 世界内联地面与光源，不依赖在线模型数据库

## 架构

```
┌────────────── myrobot.launch.xml (mock) ──────────────┐
│ robot_state_publisher → TF                              │
│ ros2_control_node（mock 硬件）→ joint_state_broadcaster │
│ ├── arm_controller     （6 轴轨迹控制器）               │
│ └── gripper_controller （双指轨迹控制器）               │
│ move_group（MoveIt 规划）＋ commander_template ＋ rviz2 │
└───────────────────────────────────────────────────────┘

┌─────────────────── gazebo.launch.xml (物理) ───────────────────┐
│ gzserver（离线世界：桌面 + 目标方块）                            │
│ └── gazebo_ros2_control 插件（gzserver 内建 controller_manager， │
│     加载同一套 3 控制器）                                        │
│ robot_state_publisher ＋ move_group ＋ rviz2                     │
│ 方块附着插件 /gazebo_grasp_attach（稳定物理抓取）                │
└────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
src/
├── myrobot_description      # 机器人模型（URDF/Xacro、ros2_control 描述、初值 YAML）
├── myrobot_moveit_config    # MoveIt 配置（SRDF/运动学/规划器/控制器映射）
├── myrobot_bringup          # mock 模式一键启动
├── myrobot_gazebo           # Gazebo 世界、抓取附着插件、启动文件
├── myrobot_commander_cpp    # 话题指挥节点、MoveIt 示例、grasp_demo
├── myrobot_interfaces       # 自定义消息 PoseCommand
└── gazebo_ros2_control      # 本地兼容补丁包（勿用 apt 覆盖，见"注意事项"）
```

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- 依赖：`ros-humble-{robot-state-publisher, rviz2, xacro, ros2-control, ros2-controllers, moveit, joint-state-publisher-gui, gazebo-ros-pkgs, gazebo-ros2-control}`

## 快速开始

```bash
# 1. 编译（必须在工作区根目录）
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. mock 仿真（终端 1）
ros2 launch myrobot_bringup myrobot.launch.xml
#    启动后等几秒，3 个控制器 active 即可接收命令

# 3. 控制机械臂（终端 2）
#    关节角控制（弧度，joint1~joint6）
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.4, 0.8, 0.0, -0.4, 0.0]}"

#    末端位姿控制（base_link 系，米/弧度；cartesian_path: true 走笛卡尔直线）
ros2 topic pub -1 /pose_command myrobot_interfaces/msg/PoseCommand \
  "{x: 0.65, y: 0.0, z: 0.63, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"

#    夹爪开合（true=张开，false=闭合）
ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: true}"

# 4. 一键抓取演示（mock 为默认模式；也可跳过第 3 步直接运行）
ros2 run myrobot_commander_cpp grasp_demo

# ── 或 Gazebo 物理仿真 ──
# 终端 1
ros2 launch myrobot_gazebo gazebo.launch.xml        # 无窗口加 gui:=false
# 终端 2（等 3 控制器 active 后再运行）
ros2 run myrobot_commander_cpp grasp_demo gazebo
```

抓取目标方块初始位于 `(0.65, 0, 0.45)`，演示将其搬运至 `(0.65, -0.157, 0.45)`；可用 `gz model -m target_box -p` 验证。

## 控制接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/joint_command` | `Float64MultiArray` | 6 关节角（弧度，joint1~joint6） |
| `/pose_command` | `PoseCommand` | 末端位姿（`base_link` 系，`cartesian_path` 可切换笛卡尔直线） |
| `/open_gripper` | `Bool` | 夹爪开合 |

- 话题控制需先运行 `commander_template`（mock 已内置；Gazebo 需手动启动）
- 直接调用控制器：`/arm_controller/follow_joint_trajectory`、`/gripper_controller/follow_joint_trajectory`（不经过 MoveIt 碰撞检查）
- 夹爪命名位姿：`gripper_open` / `gripper_close` / `gripper_half_close` / `gripper_grasp_5cm`

## 文档
- 可观看演示视频了解抓取功能
- 详细使用手册（环境准备、逐模式操作、接口示例、调试命令、故障排查）：[使用手册.md](使用手册.md)

## 注意事项

1. `src/gazebo_ros2_control` 是适配 ros2_control 2.54 的本地补丁包，**禁止用 apt 版本覆盖**（apt 升级/重装会破坏 Gazebo 链路）
2. 不要同时启动 mock 与 Gazebo（同名节点/控制器/话题冲突）
3. Gazebo 的桌面、方块与抓取坐标是一套匹配参数，修改世界尺寸需同步 `grasp_demo.cpp`
4. 停止后检查残留：`./stop_all.sh`；旧 `gzserver` 占用 11345 端口会导致新实例秒退
