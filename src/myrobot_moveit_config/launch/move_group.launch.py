# move_group.launch.py — 启动 MoveIt 运动规划节点
#
# 基于 moveit_configs_utils 的 generate_move_group_launch，额外增加
# use_sim_time 参数（默认 false）：
#   - mock 仿真（myrobot.launch.xml）：不传，保持墙钟
#   - Gazebo 仿真（gazebo.launch.xml）：传 true，与 gzserver 内 controller_manager
#     的仿真时钟对齐（否则 /joint_states 的仿真时间戳会被 move_group 判为过期，
#     轨迹执行校验失败 "couldn't receive full current joint state"）
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot", package_name="myrobot_moveit_config").to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(DeclareLaunchArgument("allow_trajectory_execution", default_value="true"))
    ld.add_action(DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true"))
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    # do not copy dynamics information from /joint_states to internal robot monitoring
    ld.add_action(DeclareLaunchArgument("monitor_dynamics", default_value="false"))
    # 仿真时钟开关：Gazebo 模式下必须为 true（见文件头注释）
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    move_group_params = [moveit_config.to_dict(), move_group_configuration]

    node_kwargs = {
        "package": "moveit_ros_move_group",
        "executable": "move_group",
        "output": "screen",
        "parameters": move_group_params,
        "additional_env": {"DISPLAY": environ.get("DISPLAY", "")},
    }

    ld.add_action(Node(**node_kwargs))
    ld.add_action(
        Node(
            **node_kwargs,
            arguments=["--debug"],
            condition=IfCondition(LaunchConfiguration("debug")),
        )
    )

    return ld
