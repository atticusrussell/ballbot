#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;


void multi_trajectory(
        moveit::planning_interface::MoveGroupInterface &dofbot_pro,
        const vector<double> &pose,
        moveit_msgs::RobotTrajectory &trajectory) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const robot_state::JointModelGroup *joint_model_group;
    // 获取机器人的起始位置
    moveit::core::RobotStatePtr start_state(dofbot_pro.getCurrentState());
    joint_model_group = start_state->getJointModelGroup(dofbot_pro.getName());
    dofbot_pro.setJointValueTarget(pose);
    dofbot_pro.plan(plan);
    start_state->setJointGroupPositions(joint_model_group, pose);
    dofbot_pro.setStartState(*start_state);
    trajectory.joint_trajectory.joint_names = plan.trajectory_.joint_trajectory.joint_names;
    for (size_t j = 0; j < plan.trajectory_.joint_trajectory.points.size(); j++) {
        trajectory.joint_trajectory.points.push_back(plan.trajectory_.joint_trajectory.points[j]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_revise_trajectory_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit_msgs::RobotTrajectory trajectory;
    moveit::planning_interface::MoveGroupInterface dofbot_pro("arm_group");
    moveit_visual_tools::MoveItVisualTools tool(dofbot_pro.getPlanningFrame());
    tool.deleteAllMarkers();

    dofbot_pro.allowReplanning(true);
    // 规划的时间(单位：秒)
    dofbot_pro.setPlanningTime(5);
    dofbot_pro.setNumPlanningAttempts(10);
    // 设置允许目标角度误差
    dofbot_pro.setGoalJointTolerance(0.01);
    dofbot_pro.setGoalPositionTolerance(0.01);
    dofbot_pro.setGoalOrientationTolerance(0.01);
    dofbot_pro.setGoalTolerance(0.01);
    // 设置允许的最大速度和加速度
    dofbot_pro.setMaxVelocityScalingFactor(1.0);
    dofbot_pro.setMaxAccelerationScalingFactor(1.0);

    // 控制机械臂先回到初始化位置
    dofbot_pro.setNamedTarget("down");
    dofbot_pro.move();

    vector<vector<double>> poses{
            {1.34,  -1.0,  -0.61, 0.2,   0},
            {0,     0,     0,     0,     0},
            {-1.16, -0.97, -0.81, -0.79, 3.14}
    };
    for (int i = 0; i < poses.size(); ++i) {
        multi_trajectory(dofbot_pro, poses.at(i), trajectory);
    }

    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    robot_trajectory::RobotTrajectory rt(dofbot_pro.getCurrentState()->getRobotModel(), "arm_group");
    rt.setRobotTrajectoryMsg(*dofbot_pro.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 1, 1);
    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;

    // 显示轨迹
    tool.publishTrajectoryLine(joinedPlan.trajectory_, dofbot_pro.getCurrentState()->getJointModelGroup("arm_group"));
    tool.trigger();

    if (!dofbot_pro.execute(joinedPlan)) {
        ROS_ERROR("Failed to execute plan");
        return false;
    }
    sleep(1);
    ROS_INFO("Finished");
    ros::shutdown();
    return 0;
}

