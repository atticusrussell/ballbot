#include <iostream>
#include "ros/ros.h"
#include <dofbot_info/kinemarics.h>
#include "dofbot_kinemarics.h"

using namespace KDL;
using namespace std;
Dofbot dofbot = Dofbot();
// 弧度转换成角度 Convert radians to degrees
const float RA2DE = 180.0f / M_PI;
// 角度转换成弧度 convert angle to radian
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/yahboom/dofbot_ws/src/dofbot_info/urdf/dofbot.urdf";
int a = 0;


bool srvicecallback(dofbot_info::kinemaricsRequest &request, dofbot_info::kinemaricsResponse &response) {
    if (request.kin_name == "fk") {
        double joints[]{request.cur_joint1, request.cur_joint2, request.cur_joint3, request.cur_joint4,
                        request.cur_joint5};
        // 定义目标关节角容器 Define the target joint angle container
        vector<double> initjoints;
        // 定义位姿容器 Define the pose container
        vector<double> initpos;
        // 目标关节角度单位转换,由角度转换成弧度 Target joint angle unit conversion, from angle to radian
        for (int i = 0; i < 5; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);
        // 调取正解函数,获取当前位姿 Call the positive solution function to get the current pose
        dofbot.dofbot_getFK(urdf_file, initjoints, initpos);
        cout << "--------- Fk ---------" << a << "--------- Fk ---------" << endl;
        cout << "XYZ coordinate： " << initpos.at(0) << " ," << initpos.at(1) << " ," << initpos.at(2) << endl;
        cout << "Roll,Pitch,Yaw： " << initpos.at(3) << " ," << initpos.at(4) << " ," << initpos.at(5) << endl;
        response.x = initpos.at(0);
        response.y = initpos.at(1);
        response.z = initpos.at(2);
        response.Roll = initpos.at(3);
        response.Pitch = initpos.at(4);
        response.Yaw = initpos.at(5);
    }
    if (request.kin_name == "ik") {
        // 夹抓长度 Grip length
        double tool_param = 0.12;
        // 抓取的位姿 grabbed pose
        double Roll = 2.5*request.tar_y*100-207.5;
        double Pitch = 0;
        double Yaw = 0;
        // 求偏移角度 find offset angle
        double init_angle = atan2(double(request.tar_x), double(request.tar_y));
        // 求夹爪在斜边的投影长度 Find the projected length of the gripper on the hypotenuse
        double dist = tool_param * sin((180 + Roll) * DE2RA);
        // 求斜边长度  Find the length of the hypotenuse
        double distance = hypot(request.tar_x, request.tar_y) - dist;
        // 求末端位置(除夹爪) Find end position (except gripper)
        double x = distance * sin(init_angle);
        double y = distance * cos(init_angle);
        double z = tool_param * cos((180 + Roll) * DE2RA);
        ///////////////  前后跟随 follow back and forth  ///////////////
        if (request.tar_z >= 0.2) {
            x=request.tar_x;
            y=request.tar_y;
            z=request.tar_z;
            Roll= -90;
        }
        // 末端位置(单位: m) End position (unit: m)
        double xyz[]{x, y, z};
        // 末端姿态(单位: 弧度) End attitude (unit: radians)
        double rpy[]{Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA};
        cout << xyz[0] << " , " << xyz[1] << " , " << xyz[2] << "\t"
        << rpy[0] << " , " << rpy[1] << " , " << rpy[2] << endl;
        // 创建输出角度容器 Create output angular container
        vector<double> outjoints;
        // 创建末端位置容器 Create end position container
        vector<double> targetXYZ;
        // 创建末端姿态容器 Create end pose container
        vector<double> targetRPY;
        for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k]);
        for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l]);
        // 反解求到达目标点的各关节角度 Inversely solve the angle of each joint to reach the target point
        dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
        // 打印反解结果 Print the inverse solution result
        cout << "--------- Ik ---------" << a << "--------- Ik ---------" << endl;
        for (int i = 0; i < 5; i++) cout << (outjoints.at(i) * RA2DE) + 90 << ",";
        cout << endl;
        a++;
        response.joint1 = (outjoints.at(0) * RA2DE) + 90;
        response.joint2 = (outjoints.at(1) * RA2DE) + 90;
        response.joint3 = (outjoints.at(2) * RA2DE) + 90;
        response.joint4 = (outjoints.at(3) * RA2DE) + 90;
        response.joint5 = (outjoints.at(4) * RA2DE) + 90;
    }
    return true;
}

/*
 * This is the ROS server for the forward and reverse solutions of the robotic arm
 * 这是机械臂正反解的ROS服务端
 * Note: The said end refers to the 5th motor rotation center, that is, the gripper is not counted
 * 注:所说的末端指的是第5个电机旋转中心,即不算夹爪
 */
int main(int argc, char **argv) {
    cout << "waiting to receive ******" << endl;
    ros::init(argc, argv, "dofbot_server");
    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("dofbot_kinemarics", srvicecallback);
    ros::spin();
    return 0;
}
