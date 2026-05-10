#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <random>

class RandomMove : public rclcpp::Node
{
public:
  RandomMove()
    : Node("random_target_move")
  {
    // 初始化其他内容
    RCLCPP_INFO(this->get_logger(), "Initializing RandomMoveIt2Control.");
  }

  void initialize()
  {
	int max_attempts = 5;  // 最大规划尝试次数
  	int attempt_count = 0;  // 当前尝试次数
    // 在此函数中初始化 move_group_interface_ 和 planning_scene_interface_
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_interface_->setNumPlanningAttempts(10);  // 设置最大规划尝试次数为 10
    move_group_interface_->setPlanningTime(5.0);         // 设置每次规划的最大时间为 5 秒
    
	moveit_msgs::msg::CollisionObject object_to_attach;
	object_to_attach.id = "cylinder1";

	shape_msgs::msg::SolidPrimitive cylinder_primitive;
	cylinder_primitive.type = cylinder_primitive.CYLINDER;
	cylinder_primitive.dimensions.resize(2);
	cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.03;
	cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.02;
	
	object_to_attach.header.frame_id = move_group_interface_->getEndEffectorLink();
	geometry_msgs::msg::Pose grab_pose;
	grab_pose.orientation.w = 1.0;
	grab_pose.position.z = 0.0;
	grab_pose.position.y = 0.075;		
	
	object_to_attach.primitives.push_back(cylinder_primitive);
	object_to_attach.primitive_poses.push_back(grab_pose);
	object_to_attach.operation = object_to_attach.ADD;
	planning_scene_interface_->applyCollisionObject(object_to_attach);	
	
	RCLCPP_INFO(this->get_logger(), "Attach the object to the robot");
	std::vector<std::string> touch_links;
	touch_links.push_back("llink2");
	touch_links.push_back("rlink2");
	move_group_interface_->attachObject(object_to_attach.id, "Gripping_point_Link", touch_links);

	
    //RCLCPP_INFO(this->get_logger(), "frame_id = %s",collision_object.header.frame_id);
    //RCLCPP_INFO(this->get_logger(), "collision_object.id = %s",collision_object.id);
    // 规划路径
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
       
    move_group_interface_->setNamedTarget("init");
    bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
    	RCLCPP_INFO(this->get_logger(), "Arm down successed.");
    	move_group_interface_->execute(my_plan);
    }
    else
    {
    	RCLCPP_ERROR(this->get_logger(), "Arm down failed!");
    }
    
    move_group_interface_->setNamedTarget("down");
    success= (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
    	RCLCPP_INFO(this->get_logger(), "Init arm successed.");
    	move_group_interface_->execute(my_plan);
    }
    else
    {
    	RCLCPP_ERROR(this->get_logger(), "Init arm failed!");
    }

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
      	RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      	move_group_interface_->execute(my_plan);
      	return;
    	}
    	else
    	{
      	RCLCPP_INFO(this->get_logger(), "Planning failed!");
    	}
	}
    
	RCLCPP_ERROR(this->get_logger(), "Exit!");

  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomMove>();
  
  // 延迟初始化
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
