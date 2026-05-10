#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>

using namespace std;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_pose_plan_cpp");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface dofbot_pro("arm_group");
    dofbot_pro.allowReplanning(true);
    // 规划的时间(单位：秒)
    dofbot_pro.setPlanningTime(5);
    dofbot_pro.setNumPlanningAttempts(10);
    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    dofbot_pro.setGoalPositionTolerance(0.01);
    dofbot_pro.setGoalOrientationTolerance(0.01);
    // 设置允许的最大速度和加速度
    dofbot_pro.setMaxVelocityScalingFactor(1.0);
    dofbot_pro.setMaxAccelerationScalingFactor(1.0);
    //dofbot_pro.setNamedTarget("init");
    //dofbot_pro.move();
//    sleep(0.1);
    //设置具体位置
    geometry_msgs::Pose pose;
    /*pose.position.x = 0.07726590386504388;
    pose.position.y = 0.0036309291391007767;
    pose.position.z = 0.4404280495807313;*/
    pose.position.x = -0.12318917780650517;
    pose.position.y = 0.21254905250348544;
    pose.position.z = 0.28040587136965994;
    /*pose.position.x = 0.1995530064718143;
    pose.position.y = 0.0004859887626293012;
    pose.position.z = 0.14950065979238644;*/
    // 设置目标姿态
    tf::Quaternion quaternion;
    // RPY的单位是角度值
    //double Roll = -180;
    //double Pitch = 45;
    //double Yaw = -180;
    // RPY转四元数
    //quaternion.setRPY(Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA);
    /*pose.orientation.x = -0.023;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.005;
    pose.orientation.w = 1.0;*/
    /*pose.orientation.x = -0.00532550583388147;
    pose.orientation.y = 0.16819564142554377;
    pose.orientation.z = -0.0009093036285580204;
    pose.orientation.w = 0.9857388286762242;*/
    pose.orientation.x = 0.3770835166718744;
    pose.orientation.y = 0.6159753747359893;
    pose.orientation.z = -0.37272863177248206;
    pose.orientation.w = 0.5826282916493523;
    

    dofbot_pro.setPoseTarget(pose);
    int index = 0;
    // 多次执行,提高成功率
    while (index <= 10) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // 运动规划
        const moveit::planning_interface::MoveItErrorCode &code = dofbot_pro.plan(plan);
        if (code == code.SUCCESS) {
            ROS_INFO_STREAM("plan success");
            dofbot_pro.execute(plan);
            break;
        } else {
            ROS_INFO_STREAM("plan error");
        }
        index++;
    }
    return 0;
}

