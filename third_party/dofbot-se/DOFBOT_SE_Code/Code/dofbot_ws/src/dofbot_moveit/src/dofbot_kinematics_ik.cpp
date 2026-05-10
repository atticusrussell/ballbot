#include "dofbot_kinemarics.h"
#include <iostream>
using namespace KDL;
using namespace std;
Dofbot dofbot = Dofbot();
// 弧度转成角度 radian to angle
const float RA2DE = 180.0f / M_PI;
// 角度转成弧度 angle to radian
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/yahboom/dofbot_ws/src/dofbot_moveit/urdf/dofbot.urdf";

/*
 * This is the code for the inverse solution of the manipulator, not involving the gripper.
 * 这是求机械臂反解的代码,不涉及夹爪.
 * Note: The joint angle obtained by the inverse solution is only data, so it may be out of bounds.
 * 注意:反解求出的关节角度只是数据,故可能会出现越界的情况.
 */
int main(int argc, char **argv) {
    // 抓取的位姿 grabbed pose
    double Roll = -135;
    double Pitch = 0;
    double Yaw = 0;
    // 求末端位置 find end position
    double x = 0;
    double y = 5.5;
    double z = 17.375;
    // 末端位置(单位: m) End position (unit: m)
    double xyz[]{x, y, z};
    // 末端姿态(单位: 弧度) End attitude (unit: radians)
    double rpy[]{Roll , Pitch, Yaw };
    // 创建输出角度容器 Create output angular container
    vector<double> outjoints;
    // 创建末端位置容器 Create end position container
    vector<double> targetXYZ;
    // 创建末端姿态容器 Create end pose container
    vector<double> targetRPY;
    // 末端位置单位转换,由cm转换成m  End position unit conversion, from cm to m
    for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k] / 100);
    // 末端姿态单位转换,由角度转换成弧度 End attitude unit conversion, convert from angle to radian
    for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l] * DE2RA);
    // 反解求到达目标点的各关节角度  Inversely solve the angle of each joint to reach the target point
    dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
    // 打印反解结果  Print the inverse solution result
    cout <<fixed<< "IK kinematics result : " << endl;
    for (int i = 0; i < 5; i++) cout << outjoints.at(i) * RA2DE + 90 << "\t";
    cout << endl;
    return 0;
}
