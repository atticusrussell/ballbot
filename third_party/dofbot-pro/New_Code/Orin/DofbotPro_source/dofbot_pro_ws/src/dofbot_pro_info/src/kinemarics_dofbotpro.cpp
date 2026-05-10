#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "dofbot_pro_interface/srv/kinemarics.hpp"
#include "dofbot_pro_info/kinemarics_dofbotpro.h"

using namespace KDL;
using namespace std;

DOFBOT_Pro dofbot_pro = DOFBOT_Pro();
int a = 0;
// 弧度转角度
const float RA2DE = 180.0f / M_PI;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf";


// auto redis = Redis("tcp://127.0.0.1:6379");
// 打印公司信息
void DOFBOT_Pro::readme(void) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Welcome to use Yahboom Technology robot.");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "www.yahboom.com");
}

// ROS 2 服务回调函数
bool srvicecallback(
    const std::shared_ptr<dofbot_pro_interface::srv::Kinemarics::Request> request,
    std::shared_ptr<dofbot_pro_interface::srv::Kinemarics::Response> response)
{
    if (request->kin_name == "fk") {
        double joints[]{request->cur_joint1, request->cur_joint2, request->cur_joint3, request->cur_joint4,
                        request->cur_joint5};
        // 定义目标关节角容器
        vector<double> initjoints;
        // 定义位姿容器
        vector<double> initpos;
        // 目标关节角度单位转换,由角度转换成弧度
        for (int i = 0; i < 5; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);
        // 调取正解函数,获取当前位姿
        dofbot_pro.dofbot_getFK(urdf_file, initjoints, initpos);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fk sent***");
        response->x = initpos.at(0);
        response->y = initpos.at(1);
        response->z = initpos.at(2);
        response->roll = initpos.at(3);
        response->pitch = initpos.at(4);
        response->yaw = initpos.at(5);
    }
    if (request->kin_name == "ik") {

       // 夹抓长度
        //double tool_param = 0.12;
        // 抓取的位姿
        double Roll = request->roll ;
        double Pitch = request->pitch;
        double Yaw = request->yaw ;
        double x=request->tar_x;
        double y=request->tar_y;
        double z=request->tar_z;
        // 末端位置(单位: m)
        double xyz[]{x, y, z};
        cout << x << y << z << endl;
        // 末端姿态(单位: 弧度)
        //double rpy[]{Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA};
    	double rpy[]{Roll , Pitch, Yaw };
        // 创建输出角度容器
        vector<double> outjoints;
        // 创建末端位置容器
        vector<double> targetXYZ;
        // 创建末端姿态容器
        vector<double> targetRPY;
        for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k]);
        for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l]);
        // 反解求到达目标点的各关节角度
        dofbot_pro.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
        // 打印反解结果
        for (int i = 0; i < 5; i++) cout << (outjoints.at(i) * RA2DE) + 90 << ",";
        cout << endl;
        a++;
        response->joint1 = (outjoints.at(0) * RA2DE) + 90;
        response->joint2 = (outjoints.at(1) * RA2DE) + 90;
        response->joint3 = (outjoints.at(2) * RA2DE) + 90;
        response->joint4 = (outjoints.at(3) * RA2DE) + 90;
        response->joint5 = (outjoints.at(4) * RA2DE) + 90;

        // redis.set("joint1", std::to_string(response->joint1));
        cout<<"-----------------"<<endl;
       
       
       
    }
    return true;
}

/*
 * 这是机械臂正反解的ROS 2服务端
 * 注:所说的末端指的是第5个电机旋转中心,即不算夹爪
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dofbot pro is waiting to receive******");
    // 创建节点
    auto server_node = rclcpp::Node::make_shared("kinemarics_dofbot");
    // 创建服务
    auto service = server_node->create_service<dofbot_pro_interface::srv::Kinemarics>(
        "dofbot_kinemarics", srvicecallback);

    // 阻塞
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
