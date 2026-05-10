#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>

using namespace std;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dofbot_motion_plan_cpp");
    ros::NodeHandle n;
    // 设置线程  set thread
    ros::AsyncSpinner spinner(1);
    // 开启线程  open thread
    spinner.start();
    // Initialize the robotic arm motion planning group
    // 初始化机械臂运动规划组
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    dofbot.allowReplanning(true);
    dofbot.setPlanningTime(5);
    dofbot.setNumPlanningAttempts(10);
    dofbot.setGoalPositionTolerance(0.01);
    dofbot.setGoalOrientationTolerance(0.01);
    dofbot.setMaxVelocityScalingFactor(1.0);
    dofbot.setMaxAccelerationScalingFactor(1.0);
    dofbot.setNamedTarget("down");
    dofbot.move();
    sleep(0.5);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0037618483876896;
    pose.position.y = 0.1128923321179022;
    pose.position.z =  0.3998656334826569;
    /*pose.position.x = -0.0025949668613384894;
    pose.position.y = 0.1115723775830233;
    pose.position.z =  0.378661833997461;*/
    tf::Quaternion quaternion;
    // RPY的单位是角度值  The unit of RPY is the angle value
    double Roll = -140;
    double Pitch = 0.0;
    double Yaw = 0.0;
    // RPY转四元数  RPY to Quaternion
    quaternion.setRPY(Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA);
    /*pose.orientation.x = -0.005464837830125983;
    pose.orientation.y = -0.002051916521318751;
    pose.orientation.z = 0.6849432014783646;
    pose.orientation.w = 0.7285730820821261;*/
    pose.orientation.x = -0.0042810851906468;
    pose.orientation.y = -0.0033330592972940;
    pose.orientation.z = 0.6827314913817025;
    pose.orientation.w = 0.7306492138509612;
    /*pose.orientation.x = -0.9746898837164969;
    pose.orientation.y = -0.0464761134797589;
    pose.orientation.z = 0.13715776530975796;
    pose.orientation.w = 0.1703154393229567;*/
    string link = dofbot.getEndEffectorLink();
    // 设置目标点  set target point
    dofbot.setPoseTarget(pose, link);
    int index = 0;
    // 多次执行,提高成功率  Execute multiple times to improve the success rate
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
    return 0;
}

