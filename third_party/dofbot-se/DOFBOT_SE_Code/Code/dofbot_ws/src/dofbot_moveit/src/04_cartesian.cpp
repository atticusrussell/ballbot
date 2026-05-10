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
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
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
    dofbot.setNamedTarget("down");
    dofbot.move();
    sleep(0.5);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0037618483876896;
    pose.position.y = 0.1128923321179022;
    pose.position.z =  0.3998656334826569;
    pose.orientation.x = -0.0042810851906468;
    pose.orientation.y = -0.0033330592972940;
    pose.orientation.z = 0.6827314913817025;
    pose.orientation.w = 0.7306492138509612;
    string link = dofbot.getEndEffectorLink();
    // 设置目标点  set target point
    dofbot.setPoseTarget(pose, link);
    int index = 0;
    while (index <= 10) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // 运动规划  motion planning
        const moveit::planning_interface::MoveItErrorCode &code = dofbot.plan(plan);
        if (code == code.SUCCESS) {
            ROS_INFO_STREAM("plan success");
            dofbot.execute(plan);
            break;
        } else {
            ROS_INFO_STREAM("plan error");
        }
        index++;
    }
    //获取当前机械臂末端位姿
    geometry_msgs::Pose start_pose = dofbot.getCurrentPose(dofbot.getEndEffectorLink()).pose;
    /*ROS_INFO("%.16f", start_pose.position.x);
    ROS_INFO("%.16f", start_pose.position.y);
    ROS_INFO("%.16f", start_pose.position.z);
    ROS_INFO("%.16f", start_pose.orientation.x);
    ROS_INFO("%.16f", start_pose.orientation.y);
    ROS_INFO("%.16f", start_pose.orientation.z);
    ROS_INFO("%.16f", start_pose.orientation.w);*/
    std::vector<geometry_msgs::Pose> waypoints;
    //将初始位姿加入路点列表
    waypoints.push_back(start_pose);  
    start_pose.position.z -= 0.01;
    waypoints.push_back(start_pose);
    start_pose.position.y -= 0.01;
    waypoints.push_back(start_pose);
    start_pose.position.z -= 0.01;
    waypoints.push_back(start_pose);
    start_pose.position.y += 0.01;
    waypoints.push_back(start_pose);
    start_pose.position.z -= 0.01;
    waypoints.push_back(start_pose);
    start_pose.position.y -= 0.01;
    waypoints.push_back(start_pose);
    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1;//0.1
    double fraction = 0.0;
    int maxtries = 1000;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    sleep(5.0);
    while (fraction < 1.0 && attempts < maxtries) {
        fraction = dofbot.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        //ROS_INFO("fraction: %f", fraction);
        attempts++;
        //if (attempts % 10 == 0) ROS_INFO("Still trying after %d attempts...", attempts);
    }
    if (fraction == 1) {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        // 显示轨迹
        tool.publishTrajectoryLine(plan.trajectory_, dofbot.getCurrentState()->getJointModelGroup("dofbot"));
        tool.trigger();
        // 执行运动
        dofbot.execute(plan);
        sleep(1);
    } else {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    return 0;
}

