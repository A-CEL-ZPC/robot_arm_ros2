#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;
class Commander 
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_,"arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_,"gripper");

        Open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper",10,std::bind(&Commander::OPenGripperCallback,this,_1));
        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command",10,std::bind(&Commander::JointCmdCallback,this,_1));
    }
    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }
    void goToJointTarget(const std::vector<double> &joint)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint);
        planAndExecute(arm_);
    }
    void goToPoseTarget(double x,double y,double z,double roll,
                        double pitch, double yaw,bool cartian_path = false)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(roll,pitch,yaw);
        qtn = qtn.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = qtn.getX();
        target_pose.pose.orientation.y = qtn.getY();
        target_pose.pose.orientation.z = qtn.getZ();
        target_pose.pose.orientation.w = qtn.getW();

        arm_->setStartStateToCurrentState();
        if(!cartian_path)
        {
            arm_->setPoseTarget(target_pose);   
            planAndExecute(arm_);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            RCLCPP_INFO(rclcpp::get_logger("test_moveit"), "笛卡尔路径完成比例: %.2f", fraction);
            if(fraction > 0.99)
            {
                arm_->execute(trajectory);
            }
        }
    }

    void open_gripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void close_gripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_close");
        planAndExecute(gripper_);
    }

private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interfaces)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interfaces->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success)
        {
            RCLCPP_INFO(node_->get_logger(), "规划成功，开始执行");
            auto exec_result = interfaces->execute(plan);
            if(exec_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "执行成功");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "执行失败，错误码: %d", exec_result.val);
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "规划失败！请确认 move_group 已启动、目标位姿可达");
        }
    }
    void OPenGripperCallback(const Bool::SharedPtr msg)
    {
        if(msg->data)
        {
            open_gripper();
        }
        else
        {
            close_gripper();
        }
    }
    void JointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;
        if(joints.size() == 6)
        {
            goToJointTarget(joints);
        }
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_; 
    rclcpp::Subscription<Bool>::SharedPtr Open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
};


int main(int argc, char ** argv)
{
    /* code */
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("Commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
