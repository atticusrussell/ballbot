#include <iostream>
#include "ros/ros.h"
#include <tf/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_joint_plan_cpp");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface dofbot_pro("arm_group");
    dofbot_pro.allowReplanning(true);
    // 规划的时间(单位：秒)
    dofbot_pro.setPlanningTime(5);
    dofbot_pro.setNumPlanningAttempts(10);
    // 设置允许目标角度误差
    dofbot_pro.setGoalJointTolerance(0.01);
    // 设置允许的最大速度和加速度
    dofbot_pro.setMaxVelocityScalingFactor(1.0);
    dofbot_pro.setMaxAccelerationScalingFactor(1.0);
    dofbot_pro.setNamedTarget("up");
    dofbot_pro.move();
//    sleep(0.1);
    //设置具体位置
    vector<double> pose{0, 0.79, -1.57, -1.57, 0};
    dofbot_pro.setJointValueTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::planning_interface::MoveItErrorCode &code = dofbot_pro.plan(plan);
    if (code == code.SUCCESS) {
        ROS_INFO_STREAM("plan success");
        // 显示轨迹
        string frame = dofbot_pro.getPlanningFrame();
        moveit_visual_tools::MoveItVisualTools tool(frame);
        tool.deleteAllMarkers();
        tool.publishTrajectoryLine(plan.trajectory_, dofbot_pro.getCurrentState()->getJointModelGroup("arm_group"));
        tool.trigger();
        dofbot_pro.execute(plan);
    } else {
        ROS_INFO_STREAM("plan error");
    }
    return 0;
}

