#include <atomic>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace myrobot_gazebo
{
class GraspAttachPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    box_model_ = std::move(model);
    world_ = box_model_->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    robot_model_name_ = sdf->HasElement("robot_model") ?
      sdf->Get<std::string>("robot_model") : "my_robot";
    robot_link_name_ = sdf->HasElement("robot_link") ?
      sdf->Get<std::string>("robot_link") : "hand_link";

    service_ = node_->create_service<std_srvs::srv::SetBool>(
      "/gazebo_grasp_attach",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        pending_.store(request->data ? 1 : 0);
        response->success = true;
        response->message = request->data ? "attach scheduled" : "detach scheduled";
      });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GraspAttachPlugin::OnUpdate, this));
    RCLCPP_INFO(node_->get_logger(), "Gazebo grasp attachment service ready");
  }

private:
  void OnUpdate()
  {
    const int request = pending_.exchange(-1);
    if (request == 1)
    {
      Attach();
    }
    else if (request == 0)
    {
      Detach();
    }

    if (attached_ && parent_link_)
    {
      box_model_->SetWorldPose(parent_link_->WorldPose() * relative_pose_);
      box_model_->SetLinearVel(ignition::math::Vector3d::Zero);
      box_model_->SetAngularVel(ignition::math::Vector3d::Zero);
    }
  }

  void Attach()
  {
    if (attached_)
    {
      return;
    }
    const auto robot = world_->ModelByName(robot_model_name_);
    const auto parent = robot ? robot->GetLink(robot_link_name_) : nullptr;
    const auto child = box_model_->GetLink("box");
    if (!parent || !child)
    {
      RCLCPP_ERROR(node_->get_logger(), "Cannot attach box: robot/link not found");
      return;
    }

    // Preserve the pose at the instant of grasp.  A physics fixed joint can
    // inject a very large impulse when the fingers are already in contact, so
    // carry the lightweight workpiece kinematically while it is attached.
    parent_link_ = parent;
    relative_pose_ = parent_link_->WorldPose().Inverse() * box_model_->WorldPose();
    const auto parent_pose = parent_link_->WorldPose();
    const auto box_pose = box_model_->WorldPose();
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Attach poses parent=(%.3f %.3f %.3f) box=(%.3f %.3f %.3f) rel=(%.3f %.3f %.3f)",
      parent_pose.Pos().X(), parent_pose.Pos().Y(), parent_pose.Pos().Z(),
      box_pose.Pos().X(), box_pose.Pos().Y(), box_pose.Pos().Z(),
      relative_pose_.Pos().X(), relative_pose_.Pos().Y(), relative_pose_.Pos().Z());
    box_model_->SetCollideMode("none");
    attached_ = true;
    RCLCPP_INFO(node_->get_logger(), "Target box attached to %s", robot_link_name_.c_str());
  }

  void Detach()
  {
    if (!attached_)
    {
      return;
    }
    box_model_->SetCollideMode("all");
    attached_ = false;
    parent_link_.reset();
    RCLCPP_INFO(node_->get_logger(), "Target box detached");
  }

  gazebo::physics::ModelPtr box_model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr parent_link_;
  ignition::math::Pose3d relative_pose_;
  bool attached_{false};
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  gazebo::event::ConnectionPtr update_connection_;
  std::atomic<int> pending_{-1};
  std::string robot_model_name_;
  std::string robot_link_name_;
};
}  // namespace myrobot_gazebo

GZ_REGISTER_MODEL_PLUGIN(myrobot_gazebo::GraspAttachPlugin)
