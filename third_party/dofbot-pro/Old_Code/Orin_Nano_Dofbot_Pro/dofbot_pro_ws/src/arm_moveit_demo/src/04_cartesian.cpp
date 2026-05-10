#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "cartesian_plan_cpp");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface dofbot("arm_group");
    string frame = dofbot.getPlanningFrame();
    moveit_visual_tools::MoveItVisualTools tool(frame);
    tool.deleteAllMarkers();
    dofbot.allowReplanning(true);
    // 规划的时间(单位：秒)
    dofbot.setPlanningTime(50);
    dofbot.setNumPlanningAttempts(10);
    // 设置允许目标角度误差
    dofbot.setGoalJointTolerance(0.001);
    dofbot.setGoalPositionTolerance(0.001);//0.01
    dofbot.setGoalOrientationTolerance(0.001);
    dofbot.setGoalTolerance(0.001);
    // 设置允许的最大速度和加速度
    dofbot.setMaxVelocityScalingFactor(1.0);
    dofbot.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO("Set Init Pose.");
    //设置具体位置
    dofbot.setNamedTarget("up");
    dofbot.move();
    sleep(0.5);
    vector<double> pose{0, -1.57, -0.74, 0.71, 0};
    dofbot.setJointValueTarget(pose);
    sleep(0.5);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    dofbot.plan(plan);
    dofbot.execute(plan);
     
    //获取当前机械臂末端位姿
    geometry_msgs::Pose start_pose = dofbot.getCurrentPose(dofbot.getEndEffectorLink()).pose;
    std::vector<geometry_msgs::Pose> waypoints;
    //将初始位姿加入路点列表
    waypoints.push_back(start_pose);
    start_pose.position.z += 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.z += 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.z += 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.z += 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.z += 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.z += 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.y -= 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.y -= 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.y -= 0.02;
   
    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1;//0.1
    double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    sleep(5.0);
    while (fraction < 1.0 && attempts < maxtries) {
        fraction = dofbot.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        //ROS_INFO("fraction: %f", fraction);
        attempts++;
        //if (attempts % 10 == 0) ROS_INFO("Still trying after %d attempts...", attempts);
    }
    ROS_INFO("fraction: %f", fraction);
    if (fraction > 0.5) {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        // 显示轨迹
        tool.publishTrajectoryLine(plan.trajectory_, dofbot.getCurrentState()->getJointModelGroup("arm_group"));
        tool.trigger();
        // 执行运动
        dofbot.execute(plan);
        sleep(1);
    } else {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    return 0;
}

