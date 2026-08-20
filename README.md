# MyRobot 六轴机械臂 ROS 2 工程

基于 **ROS 2 Humble + MoveIt 2 + ros2_control** 的六自由度机械臂运动控制工程，包含机器人建模、系统一键启动、MoveIt 运动规划与 C++ 指挥官节点。

当前阶段为**仿真验证**：硬件使用 `mock_components/GenericSystem`（虚拟硬件），可在 RViz 中完成从规划到执行的全流程。

---

## 一、系统架构

```
┌───────────────────────────── myrobot.launch.xml ─────────────────────────────┐
│                                                                              │
│  robot_state_publisher ──→ 发布 TF（从 URDF 读取 robot_description）          │
│                                                                              │
│  ros2_control_node ──→ 加载虚拟硬件 Arm，管理控制器                           │
│    ├── joint_state_broadcaster   （关节状态发布）                             │
│    ├── arm_controller            （六轴关节轨迹控制器）                       │
│    └── gripper_controller        （夹爪轨迹控制器）                           │
│                                                                              │
│  move_group（include 自 myrobot_moveit_config）──→ 运动规划                   │
│    ├── OMPL / CHOMP / Pilz 规划器 + KDL 逆解                                  │
│    └── 接收 MoveGroupInterface 的规划/执行请求                                │
│                                                                              │
│  commander_template ──→ C++ 指挥官节点（订阅 /open_gripper 话题控制夹爪）     │
│  rviz2 ──→ 可视化                                                             │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 二、目录结构

```
robot_arm_ws/
├── src/
│   ├── myrobot_description/          # 机器人模型（唯一模型源）
│   │   ├── urdf/
│   │   │   ├── myrobot_urdf.xacro          # 模型入口（6 轴机械臂 + 二指夹爪）
│   │   │   ├── arm.xacro / gripper.xacro   # 连杆与关节定义
│   │   │   ├── my_robot.ros2_control.xacro # ros2_control 硬件块（宏，初值读 YAML）
│   │   │   └── initial_positions.yaml      # 关节初始位置（唯一来源）
│   │   ├── launch/display.launch.xml       # 单独显示模型（RSP + JSP + RViz）
│   │   └── rviz/urdf_config.rviz
│   │
│   ├── myrobot_bringup/               # 系统启动包
│   │   ├── launch/myrobot.launch.xml  # 一键启动整套系统
│   │   └── config/
│   │       ├── ros2_controllers.yaml  # 控制器配置
│   │       └── myrobot_bringup.rviz   # 系统 RViz 配置
│   │
│   ├── myrobot_moveit_config/         # MoveIt 配置（MoveIt Setup Assistant 生成）
│   │   ├── config/
│   │   │   ├── my_robot.srdf          # 规划组/预设位姿/碰撞矩阵
│   │   │   ├── kinematics.yaml        # KDL 逆解
│   │   │   ├── joint_limits.yaml      # 关节速度/加速度缩放
│   │   │   ├── moveit_controllers.yaml# MoveIt ↔ 控制器映射
│   │   │   └── my_robot.urdf.xacro    # 直接引用 description 的模型
│   │   └── launch/                    # move_group / demo / rsp 等
│   │
│   └── myrobot_commander_cpp/         # C++ 指挥官包
│       ├── src/test_moveit.cpp            # 入门示例：位姿/笛卡尔/关节/命名目标
│       └── src/commander_template.cpp     # 指挥官模板：话题订阅控制夹爪开合
│
├── .vscode/c_cpp_properties.json      # VSCode C++ 智能提示（编译数据库）
└── .gitignore
```

---

## 三、环境要求

- Ubuntu 22.04 + **ROS 2 Humble**
- 已安装包：

```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit \
  ros-humble-joint-state-publisher-gui
```

> 注：`mock_components` 随 `ros2_control` 安装。

---

## 四、构建

```bash
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> ⚠️ 必须在 `robot_arm_ws` 目录下执行 `colcon build`，不要在家目录执行，否则会生成遮蔽性的 `~/install` 导致包解析错乱。

---

## 五、使用

### 1. 一键启动整套系统

```bash
ros2 launch myrobot_bringup myrobot.launch.xml
```

一次启动：robot_state_publisher、ros2_control_node、3 个控制器、move_group、commander_template、rviz2。

### 2. 话题控制夹爪开合

```bash
# 张开（注意 YAML 冒号后必须加空格）
ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: true}"

# 闭合
ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: false}"
```

launch 终端中 commander 会打印 `规划成功，开始执行` → `执行成功`。

### 3. 单独运行指挥官

```bash
ros2 run myrobot_commander_cpp commander_template
```

### 4. 单独显示机器人模型

```bash
ros2 launch myrobot_description display.launch.xml
```

### 5. 完整 MoveIt Demo（含 RViz 运动规划面板）

```bash
ros2 launch myrobot_moveit_config demo.launch.py
```

---

## 六、机器人参数

| 关节 | 类型 | 范围 |
|---|---|---|
| joint1 | revolute (Z) | ±3.14 rad |
| joint2 | revolute (Y) | 0 ~ 2.5 rad |
| joint3 | revolute (Y) | 0 ~ 2.5 rad |
| joint4 | revolute (Z) | ±3.14 rad |
| joint5 | revolute (Y) | ±1.57 rad |
| joint6 | continuous (Z) | 无限位 |
| gripper_left_finger_joint | prismatic | 0 ~ 0.06 m |
| gripper_right_finger_joint | prismatic | mimic 左指（倍率 -1） |

- 规划组：`arm`（6 关节）、`gripper`（夹爪）
- 预设位姿（SRDF group_state）：`home` / `pose_1` / `pose_2` / `gripper_open` / `gripper_close` / `gripper_half_close`
- 控制器：`joint_state_broadcaster`、`arm_controller`、`gripper_controller`（均为 joint_trajectory_controller）
- 修改初始位姿：编辑 `src/myrobot_description/urdf/initial_positions.yaml` 后重新构建

---

## 七、关键设计

1. **单一模型源**：所有包统一引用 `myrobot_description` 的 `myrobot_urdf.xacro`，`ros2_control` 硬件块由 xacro 宏生成，初值由 `initial_positions.yaml` 驱动，改初值无需改代码。
2. **仿真硬件**：`mock_components/GenericSystem` 提供位置接口的虚拟硬件，支持完整的规划-执行闭环；对接真实硬件时只需替换 URDF 中的 `<plugin>`。
3. **VSCode 智能提示**：`.vscode/c_cpp_properties.json` 指向 colcon 生成的 `compile_commands.json`，新建/修改 C++ 后先 `colcon build --packages-select myrobot_commander_cpp` 再回编辑器。

---

## 八、常见问题（FAQ）

### 1. `Unrecognized entity of the type: param` / `SyntaxError (line 1)`

- `<param>` 只能写在 `<node>` 内部，顶层 `<param>` 是 ROS 1 写法；
- 第二条 SyntaxError 是 launch 把 XML 当 Python 解析的兜底报错，忽略即可；
- 同时检查 `ros2 pkg prefix myrobot_bringup` 解析到的路径是否为当前工作区（谨防 `~/install` 等残留工作区遮蔽）。

### 2. 找不到消息头文件（如 `example_interfaces/msg/bool.hpp`）

- 确认 `CMakeLists.txt` 中同时有 `find_package(...)` **和** `ament_target_dependencies(...)`（只加前者不会添加 include 路径）；
- ROS 2 消息头文件用 **`.hpp`**（C++ 接口），`.h` 是 C 接口。

### 3. `ros2 topic pub` 报 `Failed to populate field`

YAML 里**冒号后必须有空格**：`"{data: false}"` ✅，`"{data:false}"` ❌。

### 4. rviz2 崩溃：`symbol lookup error: /snap/core20/...libpthread.so.0`

终端环境被 snap 污染（如从 snap 版 VS Code 打开终端）。检查 `echo $LD_LIBRARY_PATH` 是否含 `/snap/` 路径，换干净终端运行。

### 5. `$(command 'xacro ...')` 报 `executed command showed stderr output`

launch 的 `$(command ...)` 对 stderr 敏感，被调用的命令不能向 stderr 打印任何内容（包括弃用警告）。

### 6. 编译数据库不更新导致 VSCode 无补全

改完 C++ 代码先 `colcon build --packages-select myrobot_commander_cpp`，再 `Ctrl+Shift+P → Developer: Reload Window`。

---

## 九、开发路线

- [ ] Gazebo 物理仿真（补 `<inertial>` 惯量，`mock_components` 换 `GazeboSystem`）
- [ ] 真实硬件 hardware interface 插件（对接电机驱动）
- [ ] 视觉抓取（相机 + MoveIt 感知 + 抓取状态机）
- [ ] 完整 pick-and-place 应用
