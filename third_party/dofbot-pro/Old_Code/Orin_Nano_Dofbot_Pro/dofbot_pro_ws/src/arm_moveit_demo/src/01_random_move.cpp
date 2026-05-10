#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dofbot_pro_random_move_cpp");
    ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();
    moveit::planning_interface::MoveGroupInterface dofbot_pro("arm_group");
	// 设置最大速度
    dofbot_pro.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度
    dofbot_pro.setMaxAccelerationScalingFactor(1.0);
    //设置目标点
    dofbot_pro.setNamedTarget("down");
    //开始移动
    dofbot_pro.move();
    sleep(0.1);
    while (!ros::isShuttingDown()){
    	//设置随机目标点
    	dofbot_pro.setRandomTarget();
    	dofbot_pro.move();
    	sleep(0.5);
    }
    return 0;
}
