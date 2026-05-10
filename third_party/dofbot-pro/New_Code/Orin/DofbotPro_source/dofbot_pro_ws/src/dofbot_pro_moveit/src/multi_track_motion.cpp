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

    std::vector<double> target_joints = {1.57, -1.00, -0.61, 0.20, 0.0}; // 目标关节角度（单位：弧度）
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
      moveit::core::MoveItErrorCode execute_result = move_group_interface_->execute(my_plan);
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) 
      {
    	RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully.");
    	std::vector<double> target_joints = {0,     0,     0,     0,     0};
    	move_group_interface_->setJointValueTarget(target_joints);
    	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    	bool success_two = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    	if (success_two)
    	{
    	          // 在RViz中可视化轨迹
      		visual_tools_.publishTrajectoryLine(my_plan.trajectory_,move_group_interface_->getRobotModel()->getLinkModel("Gripping_point_Link"),joint_model_group,rviz_visual_tools::LIME_GREEN);
      		visual_tools_.trigger();
    		RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm 2nd.");
    		moveit::core::MoveItErrorCode execute_result_2nd = move_group_interface_->execute(my_plan);
    		if (execute_result_2nd == moveit::core::MoveItErrorCode::SUCCESS)
    		{
    			RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully 2nd.");
    			std::vector<double> target_joints = {-1.16, -0.50, -0.81, -0.79, 1.57};
    			move_group_interface_->setJointValueTarget(target_joints);
    			moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    			bool success_three = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    			if (success_three)
    			{
    			          // 在RViz中可视化轨迹
      				visual_tools_.publishTrajectoryLine(my_plan.trajectory_,move_group_interface_->getRobotModel()->getLinkModel("Gripping_point_Link"),joint_model_group,rviz_visual_tools::LIME_GREEN);
      				visual_tools_.trigger();
    				RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm 3rd.");
    				moveit::core::MoveItErrorCode execute_result_3rd = move_group_interface_->execute(my_plan);
    				if (execute_result_3rd == moveit::core::MoveItErrorCode::SUCCESS)
    				{
    					RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully 3rd.");
    				}
    			}
    		}  
    	}
	  } 
	  else 
	  {
    	RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed with error code: %d", execute_result.val);
	  }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
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
  auto node = std::make_shared<RandomMoveIt2Control>();
  
  // 延迟初始化
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
