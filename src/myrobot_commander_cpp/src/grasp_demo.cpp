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
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <chrono>
#include <map>
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
        // Gazebo 中保守一些，减小夹持后的惯性冲击；mock 保持原速度。
        arm_->setMaxVelocityScalingFactor(world_ == "gazebo" ? 0.25 : 0.6);
        arm_->setMaxAccelerationScalingFactor(world_ == "gazebo" ? 0.20 : 0.6);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        grasp_attach_client_ = node_->create_client<std_srvs::srv::SetBool>(
            "/gazebo_grasp_attach");
        if (world_ == "gazebo")
        {
            gripper_->setMaxVelocityScalingFactor(0.10);
            gripper_->setMaxAccelerationScalingFactor(0.10);
        }
    }

    // 主流程：逐步执行，任一步失败立即中止
    void run()
    {
        RCLCPP_INFO(node_->get_logger(), "========== 抓取任务开始 (world=%s) ==========", world_.c_str());

        if (world_ == "gazebo")
        {
            if (!setupGazeboPlanningScene()) return;

            // Gazebo 世界坐标：桌面顶 z=0.425，方块中心 (0.65, 0, 0.45)。
            // 注意：Gazebo 固定关节 lumping 会丢失 hand→tool 的 2cm 偏移，
            // 实际手指比 URDF/TF 低 2cm —— 故抓取高度取 0.51（手指实际
            // 覆盖 [0.43,0.51]，完整包住方块 [0.425,0.475]），而非 0.53。
            // 全部用笛卡尔路径：直线运动、姿态不变，避免 OMPL 选到"手腕翻转"
            // 等长路径解（会剧烈晃动把夹住的方块甩掉/挤飞）。
            // ① 先关节空间到方块上方高位，再笛卡尔垂直下降（从 home 直接
            // 笛卡尔会 0% 失败，关节空间长路径又可能扫过方块）。
            // 先在竖直 home 区域原地转腕，再走已验证的高位关节解。把 joint4 的
            // 近 π 大转动与伸臂拆开，消除 OMPL 随机路径扫过桌面的可能。
            if (!moveToJointTarget({
                {"joint1", 0.0}, {"joint2", 0.0}, {"joint3", 0.0},
                {"joint4", 3.13}, {"joint5", 0.0}, {"joint6", 0.0}}, true)) return;
            if (!moveToJointTarget({
                {"joint1", 0.0}, {"joint2", 0.271}, {"joint3", 1.918},
                {"joint4", 3.13}, {"joint5", -0.953}, {"joint6", 0.0}}, true)) return;
            // 高位已到达：切换到与 Gazebo 固定关节偏差匹配的抓取桌面几何，
            // 并移除目标方块安全包络，允许末端接近。
            if (!setGraspTableCollision()) return;
            if (!removeWorldObject("target_box")) return;
            if (!cartesianMoveToPose(0.65, 0.0, 0.63, 3.14, 0.0, 0.0)) return;   // ①b 下降到预抓取
            if (!setGripper("gripper_open")) return;                    // ② 张开
            if (!cartesianMoveToPose(0.65, 0.0, 0.51, 3.14, 0.0, 0.0)) return;   // ③ 下降到抓取位姿
            // 先建立仿真抓取约束，再播放夹爪闭合动作，避免接触求解器在锁定前
            // 将轻小方块弹出。
            if (!setPhysicalAttachment(true)) return;
            if (!setGripper("gripper_grasp_5cm")) return;               // ④ 轻夹 5cm 方块（±0.0355）
            // 等待夹爪动作收敛，再以低加速度带载起步。
            std::this_thread::sleep_for(std::chrono::seconds(1));
            arm_->setMaxVelocityScalingFactor(0.08);
            arm_->setMaxAccelerationScalingFactor(0.04);
            if (!cartesianMoveToPose(0.65, 0.0, 0.63, 3.14, 0.0, 0.0)) return;   // ⑤ 抬升
            // 保持与抓取点近似相同的径向距离，只绕基座移动，避免原 (0.55,-0.15)
            // 目标跨越 IK 分支导致笛卡尔 0%/OMPL 长路径。
            if (!cartesianMoveToPose(0.65, -0.15, 0.63, 3.14, 0.0, 0.0)) return; // ⑥ 移动到桌面上方放置位
            if (!cartesianMoveToPose(0.65, -0.15, 0.51, 3.14, 0.0, 0.0)) return; // ⑦ 下降到放置位姿
            if (!setGripper("gripper_open")) return;                    // ⑧ 张开释放
            if (!setPhysicalAttachment(false)) return;
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
    bool setPhysicalAttachment(bool attach)
    {
        if (!grasp_attach_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Gazebo 抓取固定服务不可用");
            return false;
        }
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = attach;
        auto future = grasp_attach_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready ||
            !future.get()->success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Gazebo 方块%s失败", attach ? "固定" : "释放");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        RCLCPP_INFO(node_->get_logger(), "✅ Gazebo 方块%s", attach ? "已固定" : "已释放");
        return true;
    }

    bool moveToJointTarget(const std::map<std::string, double> & target,
                           bool deterministic_ptp = false)
    {
        if (deterministic_ptp)
        {
            arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
            arm_->setPlannerId("PTP");
        }
        arm_->setStartStateToCurrentState();
        arm_->clearPoseTargets();
        if (!arm_->setJointValueTarget(target))
        {
            RCLCPP_ERROR(node_->get_logger(), "关节目标超出范围");
            arm_->setPlanningPipelineId("ompl");
            arm_->setPlannerId("");
            return false;
        }

        MoveGroupInterface::Plan plan;
        if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "安全关节中间位规划失败");
            arm_->setPlanningPipelineId("ompl");
            arm_->setPlannerId("");
            return false;
        }
        if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "安全关节中间位执行失败");
            arm_->setPlanningPipelineId("ompl");
            arm_->setPlannerId("");
            return false;
        }
        arm_->setPlanningPipelineId("ompl");
        arm_->setPlannerId("");
        RCLCPP_INFO(node_->get_logger(), "✅ 安全关节中间位到达");
        return true;
    }

    moveit_msgs::msg::CollisionObject makeBox(
        const std::string & id, const std::string & frame,
        double x, double y, double z,
        double sx, double sy, double sz) const
    {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = frame;
        object.id = id;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {sx, sy, sz};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        return object;
    }

    // 将 Gazebo 世界中会影响规划的几何体同步给 MoveIt。
    bool setupGazeboPlanningScene()
    {
        // 初段关节空间规划使用真实桌面高度并加 1 cm 上方安全裕量，防止任何连杆
        // 擦碰静态工作台。到达高位后再切换成抓取阶段的补偿几何。
        // 目标方块实际尺寸为 0.05³。初段规划使用 0.12³ 安全包络，覆盖固定关节
        // lumping 的 2 cm 末端偏差和 OMPL 插值余量；到达高位后会同步删除。
        std::vector<moveit_msgs::msg::CollisionObject> objects;
        objects.push_back(makeBox("table_top", "base_link", 0.50, 0.0, 0.405,
                                  0.60, 0.60, 0.06));
        objects.push_back(makeBox("target_box", "base_link", 0.65, 0.0, 0.45,
                                  0.12, 0.12, 0.12));

        if (!planning_scene_.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(node_->get_logger(), "无法把桌面/方块加入 MoveIt Planning Scene");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "✅ Planning Scene 已加入桌面和目标方块");
        return true;
    }

    bool setGraspTableCollision()
    {
        // Gazebo 固定关节 lumping 让真实手指相对 MoveIt/TF 高 2 cm。低位抓取阶段
        // 将规划桌面同步下移 2 cm（中心 z=0.38，顶面 z=0.405），使碰撞间隙与
        // Gazebo 实物一致，同时仍保留 5 mm 桌面间隙。
        auto table = makeBox("table_top", "base_link", 0.50, 0.0, 0.38,
                             0.60, 0.60, 0.05);
        if (!planning_scene_.applyCollisionObject(table))
        {
            RCLCPP_ERROR(node_->get_logger(), "无法切换抓取阶段桌面碰撞几何");
            return false;
        }
        return true;
    }

    // removeCollisionObjects() 通过话题异步发送；这里使用 apply 服务同步删除，
    // 避免紧接着计算接近路径时目标物仍残留在场景中。
    bool removeWorldObject(const std::string & id)
    {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "base_link";
        object.id = id;
        object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!planning_scene_.applyCollisionObject(object))
        {
            RCLCPP_ERROR(node_->get_logger(), "无法从 Planning Scene 删除 %s", id.c_str());
            return false;
        }
        return true;
    }

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
    bool cartesianMoveToPose(double x, double y, double z, double roll, double pitch,
                             double yaw, bool allow_joint_fallback = true)
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
        arm_->clearPoseTargets();
        std::vector<geometry_msgs::msg::Pose> waypoints{target.pose};
        MoveGroupInterface::Plan plan;
        moveit_msgs::msg::RobotTrajectory traj;
        moveit_msgs::msg::MoveItErrorCodes err;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, traj, true, &err);
        plan.trajectory_ = traj;
        if (fraction >= 0.90 && fraction < 0.99)
        {
            // 奇异边界附近常在最后一个插值点失败；已有轨迹仍是连续、碰撞检查过
            // 的直线路径。执行它比切换到可能手腕翻转的 OMPL 长路径更安全。
            RCLCPP_WARN(node_->get_logger(),
                "笛卡尔路径完成 %.0f%%，执行安全部分路径，避免 OMPL 回退",
                fraction * 100.0);
        }
        else if (fraction < 0.90)
        {
            if (!allow_joint_fallback)
            {
                RCLCPP_ERROR(node_->get_logger(),
                    "安全笛卡尔路径仅完成 %.0f%%，禁止 OMPL 回退 (%.2f, %.2f, %.2f)",
                    fraction * 100.0, x, y, z);
                return false;
            }
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
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_attach_client_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;
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
