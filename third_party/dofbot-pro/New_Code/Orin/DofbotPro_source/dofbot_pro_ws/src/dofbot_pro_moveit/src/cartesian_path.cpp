#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class CartesianPathPlanning : public rclcpp::Node
{
public:
  CartesianPathPlanning()
    : Node("cartesian_path_planning")
    {
      RCLCPP_INFO(this->get_logger(), "Initializing CartesianPathPlanning.");
    }

  void initialize()
  {
    // 使用 RobotModelLoader 加载机器人模型
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    
    // 初始化 MoveGroupInterface
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm_group");

    moveit_visual_tools::MoveItVisualTools visual_tools(shared_from_this(), "base_link", "arm_group", move_group_interface_->getRobotModel());

    // 定义初始位置点
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = -0.00060058;
    initial_pose.position.y = 0.0247436;
    initial_pose.position.z = 0.230899;
    initial_pose.orientation.x = -0.0165455;
    initial_pose.orientation.y = -1.45575e-05;
    initial_pose.orientation.z = 1.19822e-05;
    initial_pose.orientation.w = 0.999863;
    // initial_pose.position.x = 0.028503;
    // initial_pose.position.y = 0.111138;
    // initial_pose.position.z = 0.311743;
    // initial_pose.orientation.x = 0.707138;
    // initial_pose.orientation.y = 3.71653e-06;
    // initial_pose.orientation.z = 2.01385e-05;
    // initial_pose.orientation.w = 0.707075;


    // 将机械臂移动到初始位置点
    move_group_interface_->setPoseTarget(initial_pose);
    bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Moved to initial position successfully.");

      // 定义一系列目标位姿
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose target_pose = initial_pose;

      // 添加一些路径点
      target_pose.position.y -= 0.01;
      target_pose.position.z += 0.02;
      waypoints.push_back(target_pose);
      target_pose.position.y += 0.01;
      target_pose.position.z += 0.01;
      waypoints.push_back(target_pose);
      target_pose.position.y += 0.01;
      target_pose.position.z += 0.01;
      waypoints.push_back(target_pose);
      target_pose.position.y += 0.02;
      target_pose.position.z -= 0.02;
      waypoints.push_back(target_pose);
      target_pose.position.y += 0.01;
      target_pose.position.z += 0.02;
      waypoints.push_back(target_pose);

      // 打印所有路径点
      RCLCPP_INFO(this->get_logger(), "Waypoints:");
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        RCLCPP_INFO(this->get_logger(), "Waypoint %zu: [x: %f, y: %f, z: %f]", i, waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
      }

      // 计算笛卡尔路径
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      double jump_threshold = 0.0;
      double eef_step = 0.005;
      double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);

      if (fraction >= 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "Cartesian path planned successfully (%.2f%% achieved)", fraction * 100.0);

        // 将 moveit_msgs::msg::RobotTrajectory 转换为 robot_trajectory::RobotTrajectory
        robot_trajectory::RobotTrajectory trajectory(move_group_interface_->getRobotModel(), move_group_interface_->getName());

        // 在RViz中可视化轨迹
        visual_tools.publishTrajectoryLine(trajectory_msg, move_group_interface_->getRobotModel()->getLinkModel("Gripping_point_Link"), joint_model_group, rviz_visual_tools::LIME_GREEN);
        visual_tools.trigger();

        // 执行规划好的路径
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory_msg;
        move_group_interface_->execute(plan);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to initial position");
    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathPlanning>();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // 启动异步 Spinner
  node->initialize();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}