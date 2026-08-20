// grasp_demo.cpp — pick-and-place 抓取演示
//
// 流程（状态机顺序执行）：
//   预抓取位姿 → 张开夹爪 → 下降到抓取位姿 → 闭合夹爪 → 抬升 → 放置位姿 → 张开释放
//
// 运行前提：ros2 launch myrobot_bringup myrobot.launch.xml 已启动
// 运行方式：ros2 run myrobot_commander_cpp grasp_demo
//
// 注意：当前为 mock 仿真（无物理），"物体"是虚拟的；
//       本程序验证的是 规划→执行→任务编排 的完整闭环。
//       后续接入 Gazebo 后，同一套流程即可在物理环境中运行。

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"

#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GraspDemo
{
public:
    GraspDemo(std::shared_ptr<rclcpp::Node> node)
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
        RCLCPP_INFO(node_->get_logger(), "========== 抓取任务开始 ==========");

        if (!moveToPose(0.65, 0.0, 0.45, 3.14, 0.0, 0.0)) return;   // ① 预抓取（物体上方）
        if (!setGripper("gripper_open")) return;                     // ② 张开
        if (!moveToPose(0.65, 0.0, 0.30, 3.14, 0.0, 0.0)) return;   // ③ 下降到抓取位姿
        if (!setGripper("gripper_close")) return;                    // ④ 闭合（抓住）
        if (!moveToPose(0.65, 0.0, 0.45, 3.14, 0.0, 0.0)) return;   // ⑤ 抬升
        if (!moveToPose(0.40, -0.40, 0.42, 3.14, 0.0, 0.0)) return; // ⑥ 移动到放置位
        if (!moveToPose(0.40, -0.40, 0.32, 3.14, 0.0, 0.0)) return; // ⑦ 下降到放置位姿
        if (!setGripper("gripper_open")) return;                     // ⑧ 张开释放

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
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("grasp_demo");

    // MoveIt 服务调用需要节点持续转圈
    auto spinner = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner->add_node(node);
    std::thread spinner_thread = std::thread([spinner]() { spinner->spin(); });

    GraspDemo demo(node);
    demo.run();

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
