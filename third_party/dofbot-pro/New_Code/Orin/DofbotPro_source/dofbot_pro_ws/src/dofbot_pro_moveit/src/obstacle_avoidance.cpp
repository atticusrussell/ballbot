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
    
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_object.id = "box1";
    
    
    shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.05;
	primitive.dimensions[primitive.BOX_Y] = 0.05;
	primitive.dimensions[primitive.BOX_Z] = 0.5;
	
	geometry_msgs::msg::Pose box_pose;
	box_pose.orientation.w = 0.7071;
	box_pose.orientation.y = 0.7071;
	box_pose.position.x = 0;
	box_pose.position.y = 0.25;
	box_pose.position.z = 0.30;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	
	RCLCPP_INFO(this->get_logger(), "Add an object into the world");
	planning_scene_interface_->addCollisionObjects(collision_objects);
	
    //RCLCPP_INFO(this->get_logger(), "frame_id = %s",collision_object.header.frame_id);
    //RCLCPP_INFO(this->get_logger(), "collision_object.id = %s",collision_object.id);
    // 规划路径
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
	while (attempt_count < max_attempts)
	{
		attempt_count++;
		// 设置目标关节空间值
		move_group_interface_->setNamedTarget("up");
		    // 规划路径
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (success)
    	{
          // 在RViz中可视化轨迹
      	RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      	move_group_interface_->execute(my_plan);
      	attempt_count = 0;
      	break;
    	}
    	else
    	{
      	RCLCPP_INFO(this->get_logger(), "Planning failed!");
    	}
	}


	while (attempt_count < max_attempts)
	{
		attempt_count++;
		// 设置目标关节空间值
		move_group_interface_->setNamedTarget("down");
		    // 规划路径
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		 bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (success)
    	{
          // 在RViz中可视化轨迹
      	RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      	move_group_interface_->execute(my_plan);
      	attempt_count = 0;
      	break ;
    	}
    	else
    	{
      	RCLCPP_INFO(this->get_logger(), "Planning failed!");
    	}
	}


	while (attempt_count < max_attempts)
	{
		attempt_count++;
		// 设置目标关节空间值
		move_group_interface_->setNamedTarget("up");
		    // 规划路径
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (success)
    	{
          // 在RViz中可视化轨迹
      	RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      	move_group_interface_->execute(my_plan);
      	attempt_count = 0;
      	break;
    	}
    	else
    	{
      	RCLCPP_INFO(this->get_logger(), "Planning failed!");
    	}
	}

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
