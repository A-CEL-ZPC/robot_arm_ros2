#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});

    auto arm = moveit::planning_interface::MoveGroupInterface(node,"arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // ----------Pose goal--------------------------------------------------------
    tf2::Quaternion qtn;
    qtn.setRPY(3.14,0,0);
    qtn = qtn.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.3;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.7;
    target_pose.pose.orientation.x = qtn.getX();
    target_pose.pose.orientation.y = qtn.getY();
    target_pose.pose.orientation.z = qtn.getZ();
    target_pose.pose.orientation.w = qtn.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1=(arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(success1)
    {
        arm.execute(plan1);
    }

    // ----------cartian path--------------------------------------------------------
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += -0.2;
    waypoints.push_back(pose1);
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.2;
    waypoints.push_back(pose2);
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y += -0.2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    RCLCPP_INFO(rclcpp::get_logger("test_moveit"), "笛卡尔路径完成比例: %.2f", fraction);
    if(fraction > 0.99)
    {
        arm.execute(trajectory);
    }


    // ----------Joint goal--------------------------------------------------------
    // std::vector<double> joints = {1.5, 0.5, 0.0, 0.5, 0.0, -0.7};

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);   

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1=(arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    // if(success1)
    // {
    //     arm.execute(plan1);
    // }


    // ----------named goal-------------------------------------------------------
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1=(arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    // if(success1)
    // {
    //     arm.execute(plan1);
    // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2  =(arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    
    // if(success2)
    // {
    //     arm.execute(plan2);
    // }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
