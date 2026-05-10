#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <moveit_visual_tools/moveit_visual_tools.h>
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
class RandomMoveIt2Control : public rclcpp::Node
{
public:
  RandomMoveIt2Control()
    : Node("random_moveit2_control")
  {
    // 初始化其他内容
    RCLCPP_INFO(this->get_logger(), "Initializing RandomMoveIt2Control.");
  }

  void initialize()
  {
  	int max_attempts = 5;  // 最大规划尝试次数
  	int attempt_count = 0;  // 当前尝试次数
  
      // 使用 RobotModelLoader 加载机器人模型
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(),"robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    // 在此函数中初始化 move_group_interface_ 和 planning_scene_interface_
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm_group");
    
    moveit_visual_tools::MoveItVisualTools visual_tools_(shared_from_this(), "base_link", "arm_group",move_group_interface_->getRobotModel());

    move_group_interface_->setNumPlanningAttempts(10);  // 设置最大规划尝试次数为 10
    move_group_interface_->setPlanningTime(5.0);         // 设置每次规划的最大时间为 5 秒
    move_group_interface_->allowReplanning(true);
    move_group_interface_->setReplanAttempts(5);
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.1, 0.1);
	while (attempt_count < max_attempts)
	{
		attempt_count++;
		std::vector<double> target_joints = {0.0, 0.5, -1.5, 0.5, 0}; // 目标关节角度（单位：弧度）
		// 设置目标关节空间值
		move_group_interface_->setJointValueTarget(target_joints);
		    // 规划路径
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (success)
    	{
          // 在RViz中可视化轨迹
      	visual_tools_.publishTrajectoryLine(my_plan.trajectory_,move_group_interface_->getRobotModel()->getLinkModel("Gripping_point_Link"),joint_model_group,rviz_visual_tools::LIME_GREEN);
      	visual_tools_.trigger();
      	RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      	move_group_interface_->execute(my_plan);
      	return;
    	}
    	else
    	{
      	RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    	}
	}
    
	RCLCPP_ERROR(this->get_logger(), "Exit!");

    


  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomMoveIt2Control>();
  
  // 延迟初始化
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
