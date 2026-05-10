#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dofbot_random_move_cpp");
    ros::NodeHandle n;
    // 设置线程  set thread
    ros::AsyncSpinner spinner(1);
    // 开启线程  open thread
    spinner.start();
    // Initialize the robotic arm motion planning group
    // 初始化机械臂运动规划组
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 设置目标点 set target point
//    dofbot.setNamedTarget("down");
//    // 开始移动 start moving
//    dofbot.move();
//    dofbot.asyncMove();
//    sleep(0.1);
    while (true){
        // 设置随机目标点 Set random target points
        dofbot.setRandomTarget();
        // 开始移动 start moving
        dofbot.move();
        sleep(0.5);
    }
    // 阻塞进程 blocking process
    ros::waitForShutdown();
    return 0;
}
