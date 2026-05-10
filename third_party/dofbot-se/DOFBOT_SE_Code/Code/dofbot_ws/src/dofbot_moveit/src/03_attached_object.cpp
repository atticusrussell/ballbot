#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dofbot_attached_object_cpp");
    ros::NodeHandle n;
    // 设置线程
    ros::AsyncSpinner spinner(1);
    // 开启线程
    spinner.start();
    // Initialize the robotic arm motion planning group
    // 初始化机械臂运动规划组
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    dofbot.allowReplanning(true);
    // (单位：秒)  (unit: second)
    dofbot.setPlanningTime(5);
    dofbot.setNumPlanningAttempts(10);
    // (单位：米)  (unit: m)
    dofbot.setGoalPositionTolerance(0.01);
    // (单位：弧度)  (unit: radian)
    dofbot.setGoalOrientationTolerance(0.01);
    // 设置最大速度   set maximum speed
    dofbot.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度  set maximum acceleration
    dofbot.setMaxAccelerationScalingFactor(1.0);
    //设置目标点  set target point
    dofbot.setNamedTarget("up");
    //开始移动  start moving
    dofbot.move();
    sleep(0.1);
    // Get the name of the framework the bot is planning
    // 获取机器人正在计划的框架的名称
    string frame = dofbot.getPlanningFrame();
    // 创建场景实例  Create scene instance
    moveit::planning_interface::PlanningSceneInterface scene;
    ////////////// 添加障碍物 Add obstacles ////////////////
    vector<string> object_ids;
    scene.removeCollisionObjects(object_ids);
    // 创建检测对象容器  Create a detection object container
    vector<moveit_msgs::CollisionObject> objects;
    // 创建碰撞检测对象  Create Collision Detection Objects
    moveit_msgs::CollisionObject obj;
    // 设置障碍物的id  Set the id of the obstacle
    obj.id = "obj";
    object_ids.push_back(obj.id);
    // 障碍物的状态  the status of the obstacle
    obj.operation = obj.ADD;
    // 设置障碍物的头信息  Set the header information of the obstacle
    obj.header.frame_id = frame;
    shape_msgs::SolidPrimitive primitive;
    // 设置障碍物类型  Set the obstacle type
    primitive.type = primitive.BOX;
    // 设置障碍物维度  Set Obstacle Dimensions
    primitive.dimensions.resize(3);
    // 设置障碍物的长宽高  Set the length, width and height of obstacles
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.1;
    obj.primitives.push_back(primitive);
    geometry_msgs::Pose pose;
    // 设置障碍物的位置信息[x,y,z]  Set the position information of the obstacle [x,y,z]
    pose.position.x = 0;
    pose.position.y = 0.2;
    pose.position.z = 0.15;
    // 设置障碍物的姿态信息  Set the attitude information of obstacles
    tf::Quaternion quaternion;
    // R,P,Y的单位是角度  The units of R,P,Y are degrees
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 90.0;
    // RPY转四元数  RPY to Quaternion
    quaternion.setRPY(Roll * M_PI / 180, Pitch * M_PI / 180, Yaw * M_PI / 180);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    // 设置障碍物的位姿信息 Set the pose information of obstacles
    obj.primitive_poses.push_back(pose);
    objects.push_back(obj);
    /////////// 设置障碍物的颜色 Set the color of the obstacle ////////////////////
    // 创建检测对象的颜色容器 Create a color container for detected objects
    std::vector<moveit_msgs::ObjectColor> colors;
    // 创建颜色实例 Create a color instance
    moveit_msgs::ObjectColor color;
    // 添加需要设置颜色的id Add the id that needs to set the color
    color.id = "obj";
    // 设置RGBA值,范围[0~1] Set RGBA value, range [0~1]
    color.color.r = 0;
    color.color.g = 1.0;
    color.color.b = 0;
    color.color.a = 0.5;
    colors.push_back(color);
    // 将设置的信息添加到场景中 Add set information to the scene
    scene.applyCollisionObjects(objects, colors);
    // 设置目标点 set target point
    dofbot.setNamedTarget("down");
    // 开始移动 start moving
    dofbot.move();
    sleep(0.5);
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

