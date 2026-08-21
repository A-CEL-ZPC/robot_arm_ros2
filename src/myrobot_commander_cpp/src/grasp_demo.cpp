// grasp_demo.cpp — pick-and-place 抓取演示
//
// 流程（状态机顺序执行）：
//   预抓取位姿 → 张开夹爪 → 下降到抓取位姿 → 闭合夹爪 → 抬升 → 放置位姿 → 张开释放
//
// 运行前提：myrobot.launch.xml（mock）或 gazebo.launch.xml（gazebo）已启动
// 运行方式：ros2 run myrobot_commander_cpp grasp_demo [mock|gazebo]
//           mock   = 虚拟仿真（默认，无物理，原坐标）
//           gazebo = Gazebo 物理仿真（桌面顶 z=0.425，方块中心 (0.65,0,0.45)）
//
// 注意：mock 无物理，"物体"是虚拟的；gazebo 模式下坐标按世界文件
//       myrobot_workshop.world 计算（方块 5cm 在桌面顶 0.425 上，中心 z=0.45；
//       工具朝下 roll=π，手指相对工具 z∈[-0.10,-0.02]，故抓取时工具 z=0.53
//       使手指包住方块且不碰桌面）。

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"

#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GraspDemo
{
public:
    GraspDemo(std::shared_ptr<rclcpp::Node> node, const std::string & world)
    : world_(world)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(0.6);
        arm_->setMaxAccelerationScalingFactor(0.6);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    }

    // 主流程：逐步执行，任一步失败立即中止
    void run()
    {
        RCLCPP_INFO(node_->get_logger(), "========== 抓取任务开始 (world=%s) ==========", world_.c_str());

        if (world_ == "gazebo")
        {
            // Gazebo 世界坐标：桌面顶 z=0.425，方块中心 (0.65, 0, 0.45)。
            // 注意：Gazebo 固定关节 lumping 会丢失 hand→tool 的 2cm 偏移，
            // 实际手指比 URDF/TF 低 2cm —— 故抓取高度取 0.51（手指实际
            // 覆盖 [0.43,0.51]，完整包住方块 [0.425,0.475]），而非 0.53。
            // 全部用笛卡尔路径：直线运动、姿态不变，避免 OMPL 选到"手腕翻转"
            // 等长路径解（会剧烈晃动把夹住的方块甩掉/挤飞）。
            // ① 先关节空间到方块上方高位，再笛卡尔垂直下降（从 home 直接
            // 笛卡尔会 0% 失败，关节空间长路径又可能扫过方块）。
            if (!moveToPose(0.65, 0.0, 0.90, 3.14, 0.0, 0.0)) return;   // ①a 高位（远离方块）
            if (!cartesianMoveToPose(0.65, 0.0, 0.63, 3.14, 0.0, 0.0)) return;   // ①b 下降到预抓取
            if (!setGripper("gripper_open")) return;                    // ② 张开
            if (!cartesianMoveToPose(0.65, 0.0, 0.51, 3.14, 0.0, 0.0)) return;   // ③ 下降到抓取位姿
            if (!setGripper("gripper_half_close")) return;              // ④ 闭合（0.04≈夹住 5cm 方块）
            if (!cartesianMoveToPose(0.65, 0.0, 0.63, 3.14, 0.0, 0.0)) return;   // ⑤ 抬升
            if (!cartesianMoveToPose(0.55, -0.15, 0.63, 3.14, 0.0, 0.0)) return; // ⑥ 移动到桌面上方放置位
            if (!cartesianMoveToPose(0.55, -0.15, 0.51, 3.14, 0.0, 0.0)) return; // ⑦ 下降到放置位姿
            if (!setGripper("gripper_open")) return;                    // ⑧ 张开释放
        }
        else
        {
            // mock 虚拟世界：原坐标（无物理，验证规划-执行闭环）
            if (!moveToPose(0.65, 0.0, 0.45, 3.14, 0.0, 0.0)) return;   // ① 预抓取（物体上方）
            if (!setGripper("gripper_open")) return;                     // ② 张开
            if (!moveToPose(0.65, 0.0, 0.30, 3.14, 0.0, 0.0)) return;   // ③ 下降到抓取位姿
            if (!setGripper("gripper_close")) return;                    // ④ 闭合（抓住）
            if (!moveToPose(0.65, 0.0, 0.45, 3.14, 0.0, 0.0)) return;   // ⑤ 抬升
            if (!moveToPose(0.40, -0.40, 0.42, 3.14, 0.0, 0.0)) return; // ⑥ 移动到放置位
            if (!moveToPose(0.40, -0.40, 0.32, 3.14, 0.0, 0.0)) return; // ⑦ 下降到放置位姿
            if (!setGripper("gripper_open")) return;                     // ⑧ 张开释放
        }

        RCLCPP_INFO(node_->get_logger(), "========== 抓取任务完成！==========");
    }

private:
    // 移动到目标位姿（x/y/z + RPY），成功返回 true
    bool moveToPose(double x, double y, double z, double roll, double pitch, double yaw)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(roll, pitch, yaw);
        qtn = qtn.normalize();

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        target.pose.orientation.x = qtn.getX();
        target.pose.orientation.y = qtn.getY();
        target.pose.orientation.z = qtn.getZ();
        target.pose.orientation.w = qtn.getW();

        arm_->setStartStateToCurrentState();
        arm_->clearPoseTargets();          // 清除旧目标，避免目标混合
        arm_->setPoseTarget(target);

        MoveGroupInterface::Plan plan;
        if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划失败: 目标位姿 (%.2f, %.2f, %.2f)", x, y, z);
            return false;
        }
        if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "执行失败: 目标位姿 (%.2f, %.2f, %.2f)", x, y, z);
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "✅ 到位 (%.2f, %.2f, %.2f)", x, y, z);
        return true;
    }

    // 笛卡尔直线移动到目标位姿（保持工具姿态、无关节空间翻转解）
    bool cartesianMoveToPose(double x, double y, double z, double roll, double pitch, double yaw)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(roll, pitch, yaw);
        qtn = qtn.normalize();

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        target.pose.orientation.x = qtn.getX();
        target.pose.orientation.y = qtn.getY();
        target.pose.orientation.z = qtn.getZ();
        target.pose.orientation.w = qtn.getW();

        std::vector<geometry_msgs::msg::Pose> waypoints{target.pose};
        MoveGroupInterface::Plan plan;
        moveit_msgs::msg::RobotTrajectory traj;
        moveit_msgs::msg::MoveItErrorCodes err;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, traj, true, &err);
        plan.trajectory_ = traj;
        if (fraction < 0.99)
        {
            RCLCPP_WARN(node_->get_logger(),
                "笛卡尔路径仅完成 %.0f%% (err=%d, val=%d)，回退到关节空间规划 (%.2f, %.2f, %.2f)",
                fraction * 100.0, err.val, static_cast<int>(err.val == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION),
                x, y, z);
            arm_->setStartStateToCurrentState();
            arm_->clearPoseTargets();
            arm_->setPoseTarget(target);
            if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "规划失败: 目标位姿 (%.2f, %.2f, %.2f)", x, y, z);
                return false;
            }
        }
        if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "执行失败: 目标位姿 (%.2f, %.2f, %.2f)", x, y, z);
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "✅ 到位 (%.2f, %.2f, %.2f)", x, y, z);
        return true;
    }

    // 夹爪张开/闭合（SRDF 命名位姿）
    bool setGripper(const std::string &named_target)
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget(named_target);

        MoveGroupInterface::Plan plan;
        if (gripper_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "夹爪规划失败: %s", named_target.c_str());
            return false;
        }
        if (gripper_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "夹爪执行失败: %s", named_target.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "✅ 夹爪: %s", named_target.c_str());
        return true;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    std::string world_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 用法: grasp_demo [mock|gazebo]，默认 mock
    std::string world = "mock";
    if (argc > 1) {
        world = argv[1];
        if (world != "mock" && world != "gazebo") {
            RCLCPP_ERROR(rclcpp::get_logger("grasp_demo"), "未知 world 参数 '%s'（支持 mock/gazebo）", world.c_str());
            return 1;
        }
    }

    auto node = std::make_shared<rclcpp::Node>("grasp_demo");

    // MoveIt 服务调用需要节点持续转圈
    auto spinner = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner->add_node(node);
    std::thread spinner_thread = std::thread([spinner]() { spinner->spin(); });

    GraspDemo demo(node, world);
    demo.run();

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
