import cv2
import re
import rclpy
import subprocess
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time
from cv_bridge import CvBridge
from std_msgs.msg import String,Int16
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, Bool,Float32MultiArray
from arm_msgs.msg import ArmJoints, ArmJoint
from interfaces.action import Rot
import math
import pygame
from arm_interface.msg import CurJoints
import yaml
from concurrent.futures import Future
import psutil
from ament_index_python.packages import get_package_share_directory
import os
from threading import Thread
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from utils import large_model_interface
import threading
from rclpy.executors import MultiThreadedExecutor
import functools
import Arm_Lib
Arm = Arm_Lib.Arm_Device()
from dofbot_pro_interface.srv import Kinemarics
import math
from dofbot_pro_interface.msg import ArmJoint


import os
import signal
import subprocess
import time


pkg_path = get_package_share_directory('dofbot_pro_driver')
offset_file = os.path.join(pkg_path,'config', 'offset_value.yaml')
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
#夹取物体
Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
#追踪物体
#Arm.Arm_serial_servo_write6(90,150,12,20,90,30,1000)
class CustomActionServer(Node):
    def __init__(self):
        super().__init__("action_service_ndoe")
        # 初始化参数配置 / Initialize parameter configuration
        self.init_param_config()
        # 初始化ROS通信 / Initialize ROS communication
        self.init_ros_comunication()
        # 加载地图映射文件 / Load map mapping file
        self.load_target_points()
        # 初始化机械臂抓取功能 / Initialize arm grasping function
        self.arm_grasp_init()
        # 初始化语音合成功能 / Initialize text-to-speech synthesis function
        self.system_sound_init()
        # 初始化语言设置/Initialize language settings
        self.init_language()
        exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 40"')
        self.get_logger().info("action service started...")

    def init_param_config(self):
        """
        初始化参数配置 / Initialize parameter configuration
        """
        # 设置夹取启动文件路径 / Set the path for the grasping startup file
        pkg_share = get_package_share_directory("largemodel")
        self.map_mapping_config = os.path.join(pkg_share, "config", "map_mapping.yaml")
        # 声明参数 / Declare parameters
        self.declare_parameter("Speed_topic", "/cmd_vel")
        self.declare_parameter("use_double_llm", False)
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("useolinetts", False)
        self.declare_parameter("language", "zh")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("regional_setting", "China")
        # 获取参数值 / Get parameter values
        self.Speed_topic = (
            self.get_parameter("Speed_topic").get_parameter_value().string_value
        )
        self.use_double_llm = (
            self.get_parameter("use_double_llm").get_parameter_value().bool_value
        )
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        self.useolinetts = (
            self.get_parameter("useolinetts").get_parameter_value().bool_value
        )
        self.language = (
            self.get_parameter("language").get_parameter_value().string_value
        )
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.regional_setting = (
            self.get_parameter("regional_setting").get_parameter_value().string_value
        )
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()
        self.combination_mode = False  # 组合模式 / Combination mode
        self.interrupt_flag = False  # 打断标志 / Interrupt flag
        self.action_runing = False  # 动作执行状态 / Action execution status
        self.first_record = True  # 首次记录位置 / First record
        self.is_recording = False  # 录音状态 / Recording status
        self.IS_SAVING = False #是否正在保存图像
        self.joint6 = (
            140  # 默认机械臂六轴的初始角度 / Default angle of the six-axis arm
        )

        # 图像处理对象 / Image processing object
        self.image_msg = None
        self.bridge = CvBridge()
        # 创建模型接口客户端 / Create model interface client
        self.model_client = large_model_interface.model_interface()
        #记录当前机械臂末端的位姿
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cur_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.Dir = None
        #self.cur_pose = [0.0,0.0,0.0]
        self.cur_pose = {}
        self.cur_rm_pose = {}
        self.cur_down_pose = {}
        self.cur_name = None
        self.return_flag = False
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
        self.step = 1 #表示堆叠到第几层
        self.stack_flag = False #表示启动堆叠的开关
        self.grasp_joint = 140
        self.remove_cnt = 1
        self.arm_move_flag = False
        self.return_done = False
        self.pid_num = 0.0
        self.back_list = False

    def init_ros_comunication(self):
        """
        初始化创建ros通信对象、函数 / Initialize creation of ROS communication objects and functions
        """
        # 创建速度话题发布者 / Create velocity topic publisher
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)
        # 创建动作执行服务器，用于接受动作列表，并执行动作 / Create action execution server to accept action lists and execute actions
        self._action_server = ActionServer(
            self, Rot, "action_service", self.execute_callback
        )
        # 创建机械臂角度发布者，用于发布arm6_joints，控制机械臂 / Create arm angle publisher to publish arm6_joints and control the arm
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 100)
        # 创建关节角度发布者，用于发布arm_joint控制关节 / Create joint angle publisher to publish arm_joint and control joints
        self.SingleJoint_pub = self.create_publisher(ArmJoint, "arm_joint", 100)
        # 创建执行动作状态发布者 / Create action execution status publisher
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 3)
        # 创建发布者，发布 seewhat_handle 话题 / Create publisher to publish seewhat_handle topic
        self.seewhat_handle_pub = self.create_publisher(String, "seewhat_handle", 1)
        # 创建发布者，发布 video_handle 话题
        self.video_handle_pub = self.create_publisher(String, "video_handle", 1)
        # 创建物体位置发布者，发布待夹取物体的坐标 / Create object position publisher to publish coordinates of objects to be grasped
        self.object_position_pub = self.create_publisher(
            Int16MultiArray, "corner_xy", 1
        )
        # 创建JoyCb话题发布者，启动KCF_Tracker_ALM节点测距的功能 / Create JoyCb topic publisher to enable distance measurement functionality of KCF_Tracker_ALM node
        self.joy_pub = self.create_publisher(Bool, "JoyState", 1)
        # 创建当前机械臂关节角发布者 / Create current arm joint angle publisher
        self.pub_cur_joints = self.create_publisher(CurJoints, "Curjoints", 1)
        # 创建KCF_Tracker_ALM重置发布者 / Create KCF_Tracker_ALM reset publisher
        self.reset_pub = self.create_publisher(Bool, "reset_flag", 1)
        # 创建机械臂抓取完成话题订阅者 / Create subscriber for arm grasping completion topic
        self.largemodel_arm_done_sub = self.create_subscription(
            String, "/largemodel_arm_done", self.largemodel_arm_done_callback, 1
        )
        # 创建发布者，发布 tts_topic 主题 / Create publisher to publish tts_topic topic
        self.TTS_publisher = self.create_publisher(String, "tts_topic", 5)
        # 创建tf监听者，监听坐标变换 / Create tf listener to monitor coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建打断状态发布者 / Create interrupt status publisher
        self.interrupt_flag_pub = self.create_publisher(Bool, "interrupt_flag", 1)

        # 创建堆叠层数的发布者 / Create interrupt status publisher
        self.step_pub = self.create_publisher(Int16, "step_val", 1)
        
        # wakeup话题订阅者 / Subscribe to wakeup topic
        self.wakeup_sub = self.create_subscription(
            Bool, "wakeup", self.wakeup_callback, 5
        )
        # 图像话题订阅者 / Image topic subscriber
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 100
        )
        # 录音状态话题订阅者 / Record status topic subscriber
        self.record_status_sub = self.create_subscription(
            Bool, "record_status", self.record_status_callback, 5
        )
        #订阅当前物体的位置信息的订阅者
        self.current_pose_sub = self.create_subscription(
            Float32MultiArray, "current_pose", self.get_current_pose_callback, 5
        )        

        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        
        self.sub_arm = self.create_subscription(
            ArmJoint, 
            'TargetAngle', 
            self.arm_callback, 
            qos_profile=1000
        )

        self.arm_pub_update = self.create_publisher(
            ArmJoint, 
            'ArmAngleUpdate', 
            qos_profile=1000
        )

        self.down_joints_sub = self.create_subscription(
            CurJoints, "/down_joints", self.get_down_joints_callback, 5
        )

    def get_down_joints_callback(self,msg):
        self.cur_down_pose[self.cur_down_name] = list(msg.joints)
        self.get_logger().info(f"self.cur_down_pose:{self.cur_down_pose}")
        

    def get_current_pose_callback(self,msg):
        x = msg.data[0] + self.x_offset 
        y = msg.data[1] + self.y_offset 
        z = msg.data[2] + self.z_offset 
        self.cur_pose[self.cur_name].append(x) 
        self.cur_pose[self.cur_name].append(y) 
        self.cur_pose[self.cur_name].append(z) 
        self.get_logger().info(f"self.cur_pose:{self.cur_pose}")
        


    def arm_callback(self, msg):
        arm_joint = ArmJoint()    
        if len(msg.joints) != 0:
            self.get_logger().info(f"Received joints: {msg.joints}")
            arm_joint.joints = self.cur_joints
            for _ in range(2):
                Arm.Arm_serial_servo_write6(
                    msg.joints[0], msg.joints[1], msg.joints[2],
                    msg.joints[3], msg.joints[4], msg.joints[5],
                    time=msg.run_time
                )
                self.cur_joints = list(msg.joints)
                self.arm_pub_update.publish(arm_joint)
        else:
            self.get_logger().info(f"Moving joint {msg.id} to {msg.angle}")
            arm_joint.id = msg.id
            arm_joint.angle = msg.angle
            for _ in range(2):
                self.Arm.Arm_serial_servo_write(msg.id, msg.angle, msg.run_time)
                self.cur_joints[msg.id - 1] = msg.angle
                self.arm_pub_update.publish(arm_joint)
        


    def get_current_end_pos(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')	
        request = Kinemarics.Request()
        request.cur_joint1 = self.cur_joints[0]
        request.cur_joint2 = self.cur_joints[1]
        request.cur_joint3 = self.cur_joints[2]
        request.cur_joint4 = self.cur_joints[3]
        request.cur_joint5 = self.cur_joints[4]
        request.kin_name = "fk"
        future = self.client.call_async(request)
        future.add_done_callback(self.get_fk_respone_callback)

    def get_fk_respone_callback(self, future):
        try:
            response = future.result()
			#self.get_logger().info(f'Response received: {response.x}')
            self.CurEndPos[0] = response.x 
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z 
            self.CurEndPos[3] = response.roll
            self.CurEndPos[4] = response.pitch
            self.CurEndPos[5] = response.yaw
			
            print("self.CurEndPose: ",self.CurEndPos)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        

    def system_sound_init(self):  # 初始化系统声音相关的功能

        pkg_path = get_package_share_directory("largemodel")

        if self.regional_setting == "China":  # 如果是中国地区
            if self.useolinetts:
                model_type = "oline"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.mp3"
                )
            else:
                model_type = "local"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.wav"
                )

        elif self.regional_setting == "international":  # 如果是国际地区
            #model_type = "XUNFEI_FOR_INTERNATIONAL"
            if self.useolinetts:
                model_type = "oline"
                self.tts_out_path = os.path.join(
                pkg_path, "resources_file", "XUNFEI_TTS.mp3"
            )
            else:
                model_type = "local"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.wav"
                )
        else:
            while True:
                self.get_logger().info()(
                    'Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".'
                )
                time.sleep(1)
        # 初始化语音合成模型
        self.model_client.tts_model_init(model_type, self.language)  
        # 初始化音频播放器 / Initialize audio player
        pygame.mixer.init()
        self.stop_event = threading.Event()

    def init_language(self):
        language_list = ["zh", "en"]
        if self.language not in language_list:
            while True:
                self.get_logger().info(
                    "The language setting is incorrect. Please check the action_service'' language setting in the yahboom.yaml file"
                )
                self.get_logger().info(self.language)
                time.sleep(1)

        self.feedback_largemoel_dict = {
            "zh": {  # 中文 / Chinese
                "get_current_pose_success": "机器人反馈:get_current_pose()成功",
                "arm_up_done": "机器人反馈:执行arm_up()完成",
                "arm_down_done": "机器人反馈:执行arm_down()完成",
                "drift_done": "机器人反馈:执行drift()完成",
                "wait_done": "机器人反馈:执行wait({duration})完成",
                "arm_shake_done": "机器人反馈:执行arm_shake()完成",
                "arm_nod_done": "机器人反馈:执行arm_nod()完成",
                "arm_applaud_done": "机器人反馈:执行arm_applaud()完成",
                "grasp_obj_done": "机器人反馈:执行grasp_obj({x1},{y1},{x2},{y2})完成",
                "remove_obj_done": "机器人反馈:执行remove_obj({x1},{y1},{x2},{y2},{color})完成",
                "grasp_obj_failed": "机器人反馈:执行grasp_obj({x1},{y1},{x2},{y2})失败",
                "putdown_done": "机器人反馈:执行putdown()完成",
                "apriltag_sort_done": "机器人反馈:执行apriltag_sort({target_id})完成",
                "apriltag_sort_failed": "机器人反馈:执行apriltag_sort({target_id})失败",
                "apriltag_follow_2D_done": "机器人反馈:执行apriltag_follow_2D({target_id})完成",
                "apriltag_follow_2D_failed": "机器人反馈:取消追踪机器码",
                "apriltag_remove_higher_done": "机器人反馈:执行apriltag_remove_higher({target_high})完成",
                "apriltag_remove_higher_failed": "机器人反馈:执行apriltag_remove_higher({target_high})失败",
                "color_follow_2D_done": "机器人反馈:执行color_follow_2D({color})完成",
                "color_follow_2D_failed": "机器人反馈:执行color_follow_2D({color})失败",
                "color_remove_higher_done": "机器人反馈:执行color_remove_higher({color},{target_high})完成",
                "color_remove_higher_failed": "机器人反馈:执行color_remove_higher({color},{target_high})失败",
                "response_done": "机器人反馈：回复用户完成",
                "failure_execute_action_function_not_exists": "机器人反馈:动作函数不存在，无法执行",
                "finish": "finish",
                "multiple_done": "机器人反馈：执行{actions}完成",
                "putdown_failed": "机器人反馈:执行putdown()失败,输入参数错误",
                "light_on_done": "机器人反馈:执行light_on()完成",
                "light_off_done": "机器人反馈:执行light_off()完成",
                "beep_on_done": "机器人反馈:执行beep_on()完成",
                "beep_off_done": "机器人反馈:执行beep_off()完成",
                "adjust_joint_done" : "机器人反馈:执行adjust_joint_done()完成",
                "arm_dance_done" : "机器人反馈:执行arm_dance_done()完成",
                "gripper_open_done" : "机器人反馈:执行gripper_open_done()完成",
                "gripper_close_done" : "机器人反馈:执行gripper_close_done()完成",
                "grip_pose_done" : "机器人反馈:执行grip_pose_done()完成",
                "track_pose_done" : "机器人反馈:执行track_pose_done()完成",
                "arm_move_done" : "机器人反馈:执行arm_move_done()完成",
                "garbage_sort_done": "机器人反馈:执行garbage_sort({type_})完成",
                "garbage_sort_no_found": "没有找到该类型的垃圾",
                "change_pose_done": "机器人反馈:执行change_pose()完成",
                "change_pose_failed": "机器人反馈:执行change_pose()失败",
                "record_video_done": "机器人反馈:执行record_video()完成",
                "record_video_failed": "机器人反馈:执行record_video()失败",
                "compute_pose_done": "机器人反馈:执行compute_pose({x1},{y1},{x2},{y2})完成",
                "arm_stack_done": "机器人反馈:执行arm_stack({color})完成",
                "return_to_orin_done" : "机器人反馈:执行return_to_orin_done()完成",
                "set_pose_done" : "机器人反馈:执行set_pose_done()完成",
                "send_red_back_to_orin": "机器人反馈:接下来把红色方块进行归位",
                "send_green_back_to_orin": "机器人反馈:接下来把绿色方块进行归位",
                "send_blue_back_to_orin": "机器人反馈:接下来把蓝色方块进行归位",
                "send_yellow_back_to_orin": "机器人反馈:接下来把黄色方块放回进行归位",
                "color is in the rm_list" : "移除列表中存在该色块，在移除列表中夹取该色块",
                "color is not in the rm_list" : "移除列表中不存在该色块，判断该色块的顶部是否存在其它物体",
                "grasp_from_rm_list_done": "机器人反馈:执行grasp_from_rm_list({color})完成",
                "compute_pose_order_done": "机器人反馈:执行compute_pose_order({x1},{y1},{x2},{y2},{name},{order})完成",
                "KCF_follow_done":"机器人反馈:执行cancel_KCF_follow()完成",
                "point_to_done":"已指向目标物体",
                "garbage_sort_all_done":"已分拣完全部垃圾啦",
                "grasp_from_down_list_done": "机器人反馈:执行grasp_from_down_list({color})完成"
            },
            "en": {  # 英文 / English
                "get_current_pose_success": "Robot feedback: get_current_pose() succeeded",
                "arm_up_done": "Robot feedback: Execute arm_up() completed",
                "arm_down_done": "Robot feedback: Execute arm_down() completed",
                "drift_done": "Robot feedback: Execute drift() completed",
                "wait_done": "Robot feedback: Execute wait({duration}) completed",
                "arm_shake_done": "Robot feedback: Execute arm_shake() completed",
                "arm_nod_done": "Robot feedback: Execute arm_nod() completed",
                "arm_applaud_done": "Robot feedback: Execute arm_applaud() completed",
                "grasp_obj_done": "Robot feedback: Execute grasp_obj({x1},{y1},{x2},{y2}) completed",
                "grasp_obj_failed": "Robot feedback: Execute grasp_obj({x1},{y1},{x2},{y2}) failed",
                "putdown_done": "Robot feedback: Execute putdown() completed",
                "apriltag_sort_done": "Robot feedback: Execute apriltag_sort({target_id}) completed",
                "apriltag_sort_failed": "Robot feedback: Execute apriltag_sort({target_id}) failed",
                "apriltag_follow_2D_done": "Robot feedback: Execute apriltag_follow_2D({target_id}) completed",
                "apriltag_follow_2D_failed": "Robot feedback: Execute apriltag_follow_2D({target_id}) failed",
                "apriltag_remove_higher_done": "Robot feedback: Execute apriltag_remove_higher({target_high}) completed",
                "apriltag_remove_higher_failed": "Robot feedback: Execute apriltag_remove_higher({target_high}) failed",
                "color_follow_2D_done": "Robot feedback: Execute color_follow_2D({color}) completed",
                "color_follow_2D_failed": "Robot feedback: Execute color_follow_2D({color}) failed",
                "color_remove_higher_done": "Robot feedback: Execute color_remove_higher({color},{target_high}) completed",
                "color_remove_higher_failed": "Robot feedback: Execute color_remove_higher({color},{target_high}) failed",
                "response_done": "Robot feedback: Reply to user completed",
                "failure_execute_action_function_not_exists": "Robot feedback: Execute action function not exists",
                "finish": "finish",
                "multiple_done": "Robot feedback: Execution {actions} completed",
                "light_on_done": "Robot feedback: Execution light_on() completed",
                "light_off_done": "Robot feedback: Execution light_off() completed",
                "beep_on_done": "Robot feedback: Execution beep_on() completed",
                "beep_off_done": "Robot feedback: Execution beep_off() completed",
                "adjust_joint_done" : "Robot feedback: Execution adjust_joint_done() completed",
                "arm_dance_done" : "Robot feedback: Execution arm_dance_done() completed",
                "gripper_open_done" : "Robot feedback: Execution gripper_open_done() completed",
                "gripper_close_done" : "Robot feedback: Execution gripper_close_done() completed",
                "grip_pose_done" : "Robot feedback: Execution grip_pose_done() completed",
                "track_pose_done" : "Robot feedback: Execution track_pose_done() completed",
                "arm_move_done" : "Robot feedback: Execution arm_move_done() completed",
                "garbage_sort_done": "Robot feedback: Execution garbage_sort({type_}) completed",
                "garbage_sort_no_found": "No trash of this type was found",
                "change_pose_done": "Robot feedback: Execution change_pose() completed",
                "change_pose_failed": "Robot feedback: Execution change_pose() failed",
                "record_video_done": "Robot feedback: Execution record_video() completed",
                "record_video_failed": "Robot feedback: Execution record_video() failed",
                "compute_pose_done": "Robot feedback: Execution compute_pose({x1},{y1},{x2},{y2}) completed",
                "arm_stack_done": "Robot feedback: Execution arm_stack({color}) completed",
                "return_to_orin_done" : "Robot feedback: Execution return_to_orin_done() completed",
                "set_pose_done" : "Robot feedback: Execution set_pose_done() completed",
                "send_red_back_to_orin": "Robot feedback: Next, return the red square to its original position.",
                "send_green_back_to_orin": "Robot feedback: Next, return the green square to its original position.",
                "send_blue_back_to_orin": "Robot feedback: Next, return the blue square to its original position.",
                "send_yellow_back_to_orin": "Robot feedback: Next, return the yellow square to its original position.",
                "color is in the rm_list" : "The color block exists in the removal list..",
                "color is not in the rm_list" : "The color block does  not exist in the removal list.",
                "grasp_from_rm_list_done": "Robot feedback: Execution grasp_from_rm_list({color}) completed",
                "compute_pose_order_done": "Robot feedback: Execution compute_pose_order({x1},{y1},{x2},{y2},{name},{order}) completed",
                "KCF_follow_done":"Cancel tracking",
                "point_to_done":"The target object has been identified.",
                "garbage_sort_all_done":"All the garbage has been sorted.",
                "grasp_from_down_list_done": "Robot feedback: Execution grasp_from_down_list({color}) completed.",
                "remove_obj_done": "Robot feedback: Execution remove_obj({x1},{y1},{x2},{y2},{color}) completed.",
            },
        }

    def load_target_points(self):
        """
        加载地图映射文件 /Load map mapping file
        """
        with open(self.map_mapping_config, "r") as file:
            target_points = yaml.safe_load(file)
        self.navpose_dict = {}
        for name, data in target_points.items():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = data["position"]["x"]
            pose.pose.position.y = data["position"]["y"]
            pose.pose.position.z = data["position"]["z"]
            pose.pose.orientation.x = data["orientation"]["x"]
            pose.pose.orientation.y = data["orientation"]["y"]
            pose.pose.orientation.z = data["orientation"]["z"]
            pose.pose.orientation.w = data["orientation"]["w"]
            self.navpose_dict[name] = pose

    def arm_grasp_init(self):
        self.apriltag_sort_future = Future()
        self.apriltag_follow_2D_future = Future()
        self.apriltag_remove_higher_future = Future()
        self.color_follow_2D_future = Future()
        self.color_sort_future = Future()
        self.color_remove_higher_future = Future()
        self.grasp_obj_future = Future()
        self.garbage_sort_future = Future()
        self.change_pose_future = Future()
        self.record_video_future = Future()
        self.compute_pose_future = Future()
        self.arm_stack_future = Future()
        self.return_to_orin_future = Future()

    def record_status_callback(self, msg):
        if msg.data:
            self.is_recording = True
        else:
            self.is_recording = False

    def largemodel_arm_done_callback(self, msg):
        """
        机械臂抓取完成话题回调函数/robot arm done callback function
        用于接受机械臂抓取完成话题，并设置Future对象完成 /used to receive the topic of the robotic arm grasping completion, and set the Future object to complete
        """
        if msg.data in ["apriltag_sort_done", "apriltag_sort_failed"]:
            if not self.apriltag_sort_future.done():
                self.apriltag_sort_future.set_result(msg)
        elif msg.data in ["apriltag_follow_2D_done","apriltag_follow_2D_failed"]:
            if not self.apriltag_follow_2D_future.done():
                self.apriltag_follow_2D_future.set_result(msg)
        elif msg.data in [
            "apriltag_remove_higher_done",
            "apriltag_remove_higher_failed",
        ]:
            self.get_logger().info(f"msg.data:{msg.data}")
            if not self.apriltag_remove_higher_future.done():
                self.apriltag_remove_higher_future.set_result(msg)
        elif msg.data == "color_follow_2D_done":
            if not self.color_follow_2D_future.done():
                self.color_follow_2D_future.set_result(msg)
        elif msg.data == "color_sort_done":
            if not self.color_sort_future.done():
                self.color_sort_future.set_result(msg)
        elif msg.data == "grasp_obj_done":
            if not self.grasp_obj_future.done():
                self.grasp_obj_future.set_result(msg)
        elif msg.data in [ "color_remove_higher_done","color_remove_higher_failed"]:
            if not self.color_remove_higher_future.done():
                self.color_remove_higher_future.set_result(msg)
        elif msg.data in ["garbage_sort_done", "garbage_sort_no_found","garbage_sort_all_done"]:
            if not self.garbage_sort_future.done():
                self.garbage_sort_future.set_result(msg)
        elif msg.data in ["change_pose_done", "change_pose_failed"]:
            if not self.change_pose_future.done():
                self.change_pose_future.set_result(msg)
        elif msg.data in ["record_video_done", "record_video_failed"]:
            if not self.record_video_future.done():
                self.record_video_future.set_result(msg)
        elif msg.data ==  "compute_pose_done":
            if not self.compute_pose_future.done():
                self.compute_pose_future.set_result(msg)
        elif msg.data ==  "arm_stack_done":
            if not self.arm_stack_future.done():
                self.arm_stack_future.set_result(msg)
    def wakeup_callback(self, msg):
        """
        唤醒打断回调函数/Wake-up interrupt callback function
        用于接受唤醒信号，判断是否需要打断当前的动作、语音 /used to receive the wake-up signal, determine whether to interrupt the current action, voice
        """

        if msg.data:
            if (
                pygame.mixer.music.get_busy()  # 如果音乐正在播放/If the music is playing
            ):
                self.stop_event.set()  # 停止正在播放的音乐/Stop the music currently playing
            if (
                self.action_runing  # 如果当前有动作正在执行/If there is an action currently being
            ):
                self.interrupt_flag = True  # 置位中断标志位/Set the interruption flag
        self.check_all_process()

    def get_current_pose(self):
        """
        获取当前在全局地图坐标系下的位置 /Get the current position in the global map coordinate system
        """
        # 获取当前目标点坐标
        transform = self.tf_buffer.lookup_transform(
            "map", "base_footprint", rclpy.time.Time()
        )
        # 提取位置和姿态
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        self.navpose_dict["zero"] = pose
        # 打印记录的坐标
        position = pose.pose.position
        orientation = pose.pose.orientation
        self.get_logger().info(
            f"Recorded Pose - Position: x={position.x}, y={position.y},\
                                z={position.z},Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}"
        )
        if not self.interrupt_flag:
            self.action_status_pub("get_current_pose_success")

    def action_status_pub(self, key, **kwargs):
        """
        多语言版本的动作结果发布方法
        :param key: 文本标识
        :param**kwargs: 占位符参数
        """
        text_template = self.feedback_largemoel_dict[self.language].get(key)

        try:
            message = text_template.format(**kwargs)
        except KeyError as e:
            self.get_logger().error(f"Translation placeholder error: {e} (key: {key})")
            message = f"[Translation failed: {key}]"

        # 发布消息
        self.actionstatus_pub.publish(String(data=message))
        self.get_logger().info(f"Published message: {message}")



    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):

        arm_joint = ArmJoints()
        arm_joint.joint1 = joints[0]
        arm_joint.joint2 = joints[1]
        arm_joint.joint3 = joints[2]
        arm_joint.joint4 = joints[3]
        arm_joint.joint5 = joints[4]
        arm_joint.joint6 = joints[5]
        arm_joint.time = runtime
        self.TargetAngle_pub.publish(arm_joint)



    def pubCurrentJoints(self):
        cur_joints = CurJoints()
        cur_joints.joints = self.init_joints
        self.pub_cur_joints.publish(cur_joints)

    def arm_up(self):  # 机械臂向上
        Arm.Arm_serial_servo_write6(90,90,90,90,90,90,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_up_done")
            

    def arm_down(self):  # 机械臂向下
        Arm.Arm_serial_servo_write6(90,0,90,90,90,90,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_down_done")

    def arm_dance(self):  # 机械臂跳舞
        Arm.Arm_serial_servo_write6(90,90,90,90,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,60,120,60,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,45,135,45,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,60,120,60,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,90,90,90,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,100,80,80,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,120,60,60,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 135, 45, 45, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90,90,90,90,90,90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 20, 90, 150,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 20, 90, 150,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 130, 0, 5, 90, 0,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_dance_done")    
        Arm.Arm_serial_servo_write6(90, 120, 0, 0, 90, 30,1000)
        time.sleep(2.0)


    def wait(self, duration):
        duration = float(duration)
        time.sleep(duration)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("wait_done", duration=duration)
        

    def arm_shake(self):  # 机械臂摇头
        Arm.Arm_serial_servo_write6(90, 150, 0, 5, 90, 0,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(60, 150, 0, 5, 90, 0,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(120, 150, 0, 5, 90, 0,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(60, 150, 0, 5, 90, 0,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(120, 150, 0, 5, 90, 0,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 150, 0, 5, 90, 0,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_shake_done")
        Arm.Arm_serial_servo_write6(90, 120, 0, 0, 90, 30,1000)
        time.sleep(2.0)

    def arm_nod(self):  # 机械臂点头
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 0, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 0, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90,1000)
        time.sleep(1.0)
        Arm.Arm_serial_servo_write6(90, 150, 0, 5, 90, 0,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_nod_done")
        Arm.Arm_serial_servo_write6(90, 120, 0, 0, 90, 30,1000)
        time.sleep(2.0)

    def arm_applaud(self):  # 机械臂鼓掌   
        Arm.Arm_serial_servo_write6(90, 150, 0, 5, 90, 0,500)
        time.sleep(0.8)
        Arm.Arm_serial_servo_write(6, 180, 500)
        time.sleep(0.8)
        Arm.Arm_serial_servo_write(6, 90, 500)
        time.sleep(0.8)
        Arm.Arm_serial_servo_write(6, 180, 500)
        time.sleep(0.8)
        Arm.Arm_serial_servo_write(6, 90, 1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_applaud_done")
        Arm.Arm_serial_servo_write6(90, 120, 0, 0, 90, 30,1000)
        time.sleep(2.0)

    def grip_pose(self):  # 机械臂夹取姿态
        Arm.Arm_serial_servo_write6(90,130,0,0,90,30,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("grip_pose_done")

    def track_pose(self):  # 机械臂追踪姿态
        Arm.Arm_serial_servo_write6(90,130,0,0,90,30,1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("track_pose_done")
            
            


    def cancel_KCF_follow(self):
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/KCF_Track_Move")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/ALM_KCF_Tracker")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
        self.action_status_pub("KCF_follow_done")
        

    def track(self, x1, y1, x2, y2):
        cmd1 = "ros2 run largemodel_arm KCF_Track_Move"
        cmd2 = "ros2 run largemodel_arm ALM_KCF_Tracker"
        subprocess.Popen( 
            [
                "gnome-terminal",
                "--title=KCF_track",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=ALM_KCF_Tracker_Node",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        time.sleep(5.0) #等待ALM_KCF_Tracker_Node启动完成

        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))
        while True:
            if self.interrupt_flag:
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                self.pubSix_Arm(self.init_joints)
                return
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)

    def check_close_grasp_obj(self):
        
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/KCF_Grap_Move")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/ALM_KCF_Tracker")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)     

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)
        
        


    def grasp_obj(self, x1, y1, x2, y2):
        """
        抓取物体
        x1,y1,x2,y2: 物体外边框坐标
        """
        #Arm.Arm_serial_servo_write6(90,120,12,20,90,30,1000)
        #self.check_close_grasp_obj()
        cmd1 = "ros2 run largemodel_arm KCF_Grap_Move"
        cmd2 = "ros2 run largemodel_arm ALM_KCF_Tracker"
        # cmd3 = "ros2 run --prefix 'gdb -ex run --args' M3Pro_KCF ALM_KCF_Tracker_Node"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=ALM_KCF_Tracker",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        time.sleep(5.0) #等待ALM_KCF_Tracker_Node启动完成
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        time.sleep(2.0)
        if self.stack_flag == True:
            self.get_logger().info('Publish the stack_step topic...') 
            step_ = Int16()
            step_.data = self.step
            self.step_pub.publish(step_)
            self.step = self.step + 1
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))

        while not self.grasp_obj_future.done():
            if self.interrupt_flag:
                self.check_close_grasp_obj()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.grasp_obj_future.result()
        
        if not self.interrupt_flag:
            if result.data == "grasp_obj_done":
                self.action_status_pub("grasp_obj_done", x1=x1, y1=y1, x2=x2, y2=y2)
            else:
                self.action_status_pub("grasp_obj_failed", x1=x1, y1=y1, x2=x2, y2=y2)
        self.check_close_grasp_obj()
        self.grasp_obj_future = Future()  # 复位Future对象
        if self.interrupt_flag:
            time.sleep(0.5)
            Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
            #self.pubSix_Arm(self.init_joints)  # 机械臂收回





    def check_close_remove_obj(self):
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/KCF_Grap_Move")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/ALM_KCF_Tracker")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)


    def remove_obj(self, x1, y1, x2, y2,color):
        cur_rm_name = color.strip("'\"")  # 去掉单引号和双引号
        self.cur_rm_name = cur_rm_name
        self.cur_rm_pose[self.cur_rm_name] = []
        self.remove_cnt = -self.remove_cnt 
        """
        抓取物体
        x1,y1,x2,y2: 物体外边框坐标
        """
        #Arm.Arm_serial_servo_write6(90,120,12,20,90,30,1000)
        #self.check_close_remove_obj()
        cmd1 = "ros2 run largemodel_arm KCF_Grap_Move"
        cmd2 = "ros2 run largemodel_arm ALM_KCF_Tracker"
        # cmd3 = "ros2 run --prefix 'gdb -ex run --args' M3Pro_KCF ALM_KCF_Tracker_Node"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=ALM_KCF_Tracker",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        time.sleep(5.0) #等待ALM_KCF_Tracker_Node启动完成
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        time.sleep(2.0)
        if self.stack_flag == True:
            self.get_logger().info('Publish the stack_step topic...') 
            step_ = Int16()
            step_.data = self.step
            self.step_pub.publish(step_)
            self.step = self.step + 1
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))

        while not self.grasp_obj_future.done():
            if self.interrupt_flag:
                self.check_close_grasp_obj()
                return
            time.sleep(0.1)
        self.check_close_remove_obj()
        

        tmp_joint1 = 90 + self.remove_cnt*60 #30 150
        self.cur_rm_pose[self.cur_rm_name] = [tmp_joint1,50,40,20,90,30]
        self.get_logger().info(f"self.cur_rm_pose:{self.cur_rm_pose}")
        Arm.Arm_serial_servo_write6(tmp_joint1,50,40,20,90,140,2000)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write6(tmp_joint1,50,40,20,90,30,2000)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,2000)
        time.sleep(2.0)
        self.action_status_pub("remove_obj_done", x1=x1, y1=y1, x2=x2, y2=y2, color=color)

        self.grasp_obj_future = Future()  # 复位Future对象
        if self.interrupt_flag:
            time.sleep(0.5)
            Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
            #self.pubSix_Arm(self.init_joints)  # 机械臂收回




    def putdown(self):
        Arm.Arm_serial_servo_write6(180, 90, 28, 5, 90, 145,1000) 
        time.sleep(2)
        Arm.Arm_serial_servo_write(6, 0, 1000)  # 机械臂打开夹抓，放下物品
        time.sleep(3)
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000) # 机械臂收回
        time.sleep(2.0)
        if not self.interrupt_flag:
            self.action_status_pub("putdown_done")


    def seewhat(self):
        self.save_single_image()
        time.sleep(3.0)
        msg = String(data="seewhat")
        self.seewhat_handle_pub.publish(
            msg
        )  # 归一化，发布seewhat话题，由model_service调用大模型


    def video_understanding(self):
        self.get_logger().info(
                    "Publish video handle."
                )
        msg = String(data="video_handle")
        self.video_handle_pub.publish(
            msg
        )  # 归一化，发布seewhat话题，由model_service调用大模型


    def _execute_action(self, twist, num=1, durationtime=3.0):
        for _ in range(num):
            start_time = time.time()
            while (time.time() - start_time) < durationtime:
                if self.interrupt_flag:
                    Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                    return
                self.publisher.publish(twist)
                time.sleep(0.1)

    def check_apriltag_sort(self):

        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/grasp")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/apriltag_sort")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)     

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)

        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
        

    def apriltag_sort(self, target_id):  # 夹取机器码
        #self.check_apriltag_sort()
        target_idf = float(target_id)
        cmd1 = "ros2 run largemodel_arm grasp"
        cmd2 = f"ros2 run largemodel_arm apriltag_sort --ros-args -p target_id:={target_idf:.1f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_apritag",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=apriltag_sort",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.apriltag_sort_future.done():
            if self.interrupt_flag:
                self.check_apriltag_sort()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.apriltag_sort_future.result()
        self.check_apriltag_sort()
        if not self.interrupt_flag:
            if result.data == "apriltag_sort_done":
                self.action_status_pub("apriltag_sort_done", target_id=target_id)
            elif result.data == "apriltag_sort_failed":
                self.action_status_pub("apriltag_sort_failed", target_id=target_id)

        
        self.apriltag_sort_future = Future()  # 复位Future对象

    def check_apriltag_remove_higher(self):
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/grasp")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/apriltag_remove_higher")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)     

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)

        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)

    def apriltag_remove_higher(self, target_high):  # 移除指定高度的机器码
        target_highf = float(target_high)
        cmd1 = "ros2 run largemodel_arm grasp"
        cmd2 = f"ros2 run largemodel_arm apriltag_remove_higher --ros-args -p target_high:={target_highf:.2f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_remove",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=apriltag_remove_higher",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.apriltag_remove_higher_future.done():
            if self.interrupt_flag:
                self.check_apriltag_remove_higher()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)
        result = self.apriltag_remove_higher_future.result()
        self.check_apriltag_remove_higher()
        if not self.interrupt_flag:
            if result.data == "apriltag_remove_higher_done":
                self.action_status_pub(
                    "apriltag_remove_higher_done", target_high=target_high
                )
            elif result.data == "apriltag_remove_higher_failed":
                self.action_status_pub(
                    "apriltag_remove_higher_failed", target_high=target_high
                )

        
        self.apriltag_remove_higher_future = Future()  # 复位Future对象

    def check_color_remove_higher(self):
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/grasp")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/color_remove_higher")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)     

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)

        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
    

            

    def color_remove_higher(self, color, target_high):
        arm_joints = [90, 110, 0, 0, 90, 0]
        self.pubSix_Arm(arm_joints)
        color = color.strip("'\"")  # 去掉单引号和双引号
        target_highf = float(target_high) / 10
        if color == "red":
            target_color = float(1)
        elif color == "green":
            target_color = float(2)
        elif color == "blue":
            target_color = float(3)
        elif color == "yellow":
            target_color = float(4)
        else:
            self.get_logger().info(
                "Fatal ERROR:Incorrect color input,Does the AI output not meet expectations?"
            )
            self.action_status_pub(
                "color_remove_higher_failed", color=color, target_high=target_high
            )
            return
        
        cmd1 = "ros2 run largemodel_arm grasp"
        cmd2 = f"ros2 run largemodel_arm color_remove_higher --ros-args -p target_high:={target_highf:.2f} -p target_color:={target_color:.1f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_remove_color",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=color_remove_higher",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.color_remove_higher_future.done():
            if self.interrupt_flag:
                self.check_color_remove_higher()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.color_remove_higher_future.result()
        self.check_color_remove_higher()
        if not self.interrupt_flag:
            if result.data == "color_remove_higher_done":
                self.action_status_pub(
                    "color_remove_higher_done", color=color, target_high=target_high
                )
            else:
                self.action_status_pub(
                    "color_remove_higher_failed", color=color, target_high=target_high
                )

        
        self.color_remove_higher_future = Future()  # 复位Future对象
        self.pubSix_Arm(self.init_joints)


    def check_garbage_sort(self):
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/yolov11_ALM")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/dofbot_pro_ws/install/dofbot_pro_yolov11/lib/dofbot_pro_yolov11/yolov11_sortation")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)   

        pid_num = self.get_pid_by_command("/home/jetson/dofbot_pro_ws/install/dofbot_pro_yolov11/lib/dofbot_pro_yolov11/msgToimg")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM) 

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-3], signal.SIGKILL)
        time.sleep(2.0)

        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
                           
    def garbage_sort(self, type_):
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
        type_ = type_.strip("'\"")  # 去掉单引号和双引号
        if type_ == "rec":
            target_type_ = float(1)
        elif type_ == "tox":
            target_type_ = float(2)
        elif type_ == "wet":
            target_type_ = float(3)
        elif type_ == "dry":
            target_type_ = float(4)
        else:
            self.get_logger().info(
                "Fatal ERROR:Incorrect type_ input,Does the AI output not meet expectations?"
            )
            self.action_status_pub(
                "garbage_sort_failed", type_=type_
            )
            return
        
        cmd1 = "ros2 run dofbot_pro_yolov11 yolov11_sortation"
        cmd2 = "ros2 run dofbot_pro_yolov11 msgToimg"
        cmd3 = f"ros2 run largemodel_arm yolov11_ALM --ros-args  -p target_type:={target_type_:.1f}"
        
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_garbage_sort",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=detect_img",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=garbage_detect",
                "--",
                "bash",
                "-c",
                f"{cmd3}; exec bash",
            ]
        )

        while not self.garbage_sort_future.done():
            if self.interrupt_flag:
                self.check_garbage_sort()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.garbage_sort_future.result()
        self.check_garbage_sort()
        if not self.interrupt_flag:
            if result.data == "garbage_sort_done":
                self.action_status_pub(
                    "garbage_sort_done", type_=type_
                )
            elif result.data == "garbage_sort_no_found":
                self.action_status_pub(
                    "garbage_sort_no_found"
                )

        
        self.garbage_sort_future = Future()  # 复位Future对象




    def check_close_change_pose(self):
        self.get_logger().info('Checking...')
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/Get_Target_Pose_KCF")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM) 

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)



    def change_pose(self, x1, y1, x2, y2,x3,y3,x4,y4,src,tar,side):
        src_color = src.strip("'\"")  # 去掉单引号和双引号
        self.cur_down_name = src_color
        self.cur_down_pose[self.cur_down_name] = []
        cmd1 = "ros2 run largemodel_arm Get_Target_Pose_KCF"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        time.sleep(3.0)
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        x3 = int(x3)
        y3 = int(y3)
        x4 = int(x4)
        y4 = int(y4)
        side = int(side)
        time.sleep(5.0)       
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2,x3,y3,x4,y4,side]))

        while not self.change_pose_future.done():
            if self.interrupt_flag:
                self.check_close_change_pose()
                #self.pubSix_Arm(self.init_joints)  # 机械臂收回
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.change_pose_future.result()
        self.check_close_change_pose()
        if not self.interrupt_flag:
            if result.data == "change_pose_done":
                self.action_status_pub("change_pose_done", x1=x1, y1=y1, x2=x2, y2=y2)
            else:
                self.action_status_pub("change_pose_failed", x1=x1, y1=y1, x2=x2, y2=y2)
        
        self.change_pose_future = Future()  # 复位Future对象
        if self.interrupt_flag:
            time.sleep(0.5)
            Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)


    def check_close_record_video(self):
        """
        检查相关进程是否存活
        """
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "recording"], check=False, timeout=2
            )
        except subprocess.TimeoutExpired:
            # 如果关闭窗口超时，尝试强制杀死进程
            subprocess.run(["pkill", "-f", "recording"])


    

    def record_video(self,time_):
        #self.check_close_record_video()
        record_time_ = float(time_)
        cmd1 = f"ros2 run largemodel_arm Record_Video --ros-args -p time_:={record_time_:.2f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=recording",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )        
        
        while not self.record_video_future.done():
            if self.interrupt_flag:
                self.check_close_record_video()
                #self.pubSix_Arm(self.init_joints)  # 机械臂收回
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.record_video_future.result()
        
        if not self.interrupt_flag:
            if result.data == "record_video_done":
                self.action_status_pub("record_video_done", time_=time_)
            else:
                self.action_status_pub("record_video_failed", time_=time_)
        self.check_close_record_video()
        self.record_video_future = Future()  # 复位Future对象



    def check_close_apriltag_follow(self):
        """
        检查相关进程是否存活
        """
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_follow"], check=False, timeout=2
            )
        except subprocess.TimeoutExpired:
            # 如果关闭窗口超时，尝试强制杀死进程
            subprocess.run(["pkill", "-f", "recording"])


    

    def apriltag_follow(self,target_id):
        target_idf = float(target_id)
        cmd1 = f"ros2 run largemodel_arm apriltag_follow_2D --ros-args -p target_id:={target_idf:.1f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=apriltag_follow",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )        
        
        while not self.record_video_future.done():
            if self.interrupt_flag:
                self.check_close_record_video()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.record_video_future.result()
        
        if not self.interrupt_flag:
            if result.data == "apriltag_follow_2D_done":
                self.action_status_pub("apriltag_follow_2D_done", target_id=target_id)
            else:
                self.action_status_pub("apriltag_follow_2D_failed")

        self.apriltag_follow_2D_future = Future()  # 复位Future对象



    def cancel_apriltag_follow(self,target_id):
        self.get_logger().info('Checking...')
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/apriltag_follow_2D")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM) 

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        self.action_status_pub("apriltag_follow_2D_failed")
           


    def kill_all_bash_processes(self):
        """杀死所有bash进程"""
        killed = []
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                # 只处理进程名为bash的进程
                if proc.info['name'] == 'bash':
                    pid = proc.info['pid']
                    os.kill(pid, signal.SIGKILL)  # 发送kill -9信号
                    killed.append(pid)
                    print(f"已杀死bash进程：{pid}")
            except (psutil.NoSuchProcess, psutil.AccessDenied, OSError) as e:
                print(f"跳过进程 {proc.info.get('pid')}：{e}")
        
        if killed:
            print(f"总计杀死 {len(killed)} 个bash进程")
        else:
            print("未找到bash进程")
    
    def get_pid_by_command(self,command_keyword):
        """根据命令关键字查找进程PID"""
        pids = []
        for proc in psutil.process_iter(['pid', 'cmdline']):
            try:
                # 获取进程的命令行列表
                cmdline = proc.info['cmdline']
                if cmdline:
                    # 将命令行列表拼接成字符串，便于查找关键字
                    cmd_str = ' '.join(cmdline)
                    if command_keyword in cmd_str:
                        pids.append(proc.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return pids


    def get_bash_pid_by_tty(self,tty_name):
        """
        根据终端名称（如pts/5）获取对应的bash进程PID
        :param tty_name: 终端路径（如"pts/5"）
        :return: 匹配的PID，未找到则返回None
        """
        self.get_logger().info(f"tty_name:{tty_name}")
        for proc in psutil.process_iter(['pid', 'name', 'terminal']):
            try:
                # 筛选条件：进程名为bash，且终端为pts/5
                if  proc.info['terminal'] == tty_name:
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None


    def get_ss_plus_bash_pids(self):
        """获取状态为Ss+且进程名为bash的PID"""
        try:
            # 使用ps命令获取所有进程的状态、PID和命令名
            result = subprocess.check_output(
                ['ps', '-e', '-o', 'stat,pid,cmd'],
                text=True,
                stderr=subprocess.STDOUT
            )
            
            # 正则匹配：状态严格为Ss+，命令包含bash（排除grep干扰）
            # 状态字段严格匹配"Ss+"，中间是PID，命令包含bash
            pattern = re.compile(
                r'^Ss\+\s+(\d+)\s+.*bash(?!.*grep)',  # 严格匹配Ss+状态
                re.MULTILINE
            )
            pids = pattern.findall(result)
            
            return [int(pid) for pid in pids]
            
        except subprocess.CalledProcessError as e:
            print(f"执行ps命令出错: {e.output}")
            return []
        except Exception as e:
            print(f"发生错误: {e}")
            return []

    

    
        
    def check_close_compute_pose(self):
        
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/Record_pose")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)
        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        
  

    def compute_pose(self,x1,y1,x2,y2,name):
        cur_name = name.strip("'\"")  # 去掉单引号和双引号
        self.cur_name = cur_name
        self.cur_pose[self.cur_name] = []
        #self.check_close_compute_pose()
        cmd1 = "ros2 run largemodel_arm Record_pose"
        # cmd3 = "ros2 run --prefix 'gdb -ex run --args' M3Pro_KCF ALM_KCF_Tracker_Node"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=compute_pose",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
                        
        time.sleep(5.0)
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))


        while not self.compute_pose_future.done():
            if self.interrupt_flag:
                self.check_compute_pose()
                #self.pubSix_Arm(self.init_joints)  # 机械臂收回
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.compute_pose_future.result()
        
        if not self.interrupt_flag:
            if result.data == "compute_pose_done":
                self.action_status_pub("compute_pose_done", x1=x1, y1=y1, x2=x2, y2=y2)
            else:
                self.action_status_pub("compute_pose_failed", x1=x1, y1=y1, x2=x2, y2=y2)
        self.check_close_compute_pose()
        
        self.compute_pose_future = Future()




    def compute_pose_order(self,x1,y1,x2,y2,name,order):
        cur_name = name.strip("'\"")  # 去掉单引号和双引号
        color_order = order.strip("'\"")
        order_list = list(color_order[1:])
        self.get_logger().info(f"order_list:{order_list}")
        self.cur_name = cur_name
        self.cur_pose[self.cur_name] = []
        #self.check_close_compute_pose()
        cmd1 = "ros2 run largemodel_arm Record_pose"
        # cmd3 = "ros2 run --prefix 'gdb -ex run --args' M3Pro_KCF ALM_KCF_Tracker_Node"
        proc = subprocess.Popen(
            [
                "gnome-terminal",
                "--title=compute_pose",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        self.pid_num = proc.pid   
        self.get_logger().info(f"self.pid_num:{self.pid_num}")
        time.sleep(5.0)
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))

        while not self.compute_pose_future.done():
            if self.interrupt_flag:
                self.check_close_compute_pose()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.compute_pose_future.result()
        self.check_close_compute_pose()
        if not self.interrupt_flag:
            if result.data == "compute_pose_done":
                for i in range(0,len(order_list)):
                    x = self.cur_pose[cur_name][0]
                    y = self.cur_pose[cur_name][1]
                    z = self.cur_pose[cur_name][2] - (i+1)*0.028
                    if order_list[i] == 'r' or  order_list[i] == 'R':
                        self.cur_pose['red'] = []
                        self.cur_pose['red'].append(x) 
                        self.cur_pose['red'].append(y) 
                        self.cur_pose['red'].append(z) 
                    elif order_list[i] == 'b' or  order_list[i] == 'B':
                        self.cur_pose['blue'] = []
                        self.cur_pose['blue'].append(x) 
                        self.cur_pose['blue'].append(y) 
                        self.cur_pose['blue'].append(z)                
                    elif order_list[i] == 'y' or  order_list[i] == 'Y':
                        self.cur_pose['yellow'] = []
                        self.cur_pose['yellow'].append(x) 
                        self.cur_pose['yellow'].append(y) 
                        self.cur_pose['yellow'].append(z)  
                    elif order_list[i] == 'g' or  order_list[i] == 'G':
                        self.cur_pose['green'] = []
                        self.cur_pose['green'].append(x) 
                        self.cur_pose['green'].append(y) 
                        self.cur_pose['green'].append(z)  
                        
                self.get_logger().info(f"self.cur_pose:{self.cur_pose}")
                self.action_status_pub("compute_pose_order_done", x1=x1, y1=y1, x2=x2, y2=y2,name=name,order=order)
            else:
                self.action_status_pub("compute_pose_failed", x1=x1, y1=y1, x2=x2, y2=y2)


    
        #time.sleep(2.0)
        #Arm.Arm_Buzzer_On()
        #time.sleep(1.0)
        #Arm.Arm_Buzzer_Off()
        
        self.compute_pose_future = Future()


    def return_to_orin(self,name):
        tar_name = name.strip("'\"")
        self.get_logger().info(f'"tar_name": {tar_name}')
        self.return_flag = True
        self.get_logger().info('Start to put it to the orin position.')
        self.get_current_end_pos()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')     
        if tar_name in self.cur_pose:
            self.get_logger().info('Get the orin position.') 
            request = Kinemarics.Request()
            request.tar_x = self.cur_pose[tar_name][0]
            request.tar_y = self.cur_pose[tar_name][1] + 0.01
            request.tar_z = self.cur_pose[tar_name][2] + 0.01
            request.kin_name = "ik"
            request.roll = -1.0
            #request.pitch = self.CurEndPos[4]
            #request.yaw = self.CurEndPos[5]
            self.get_logger().info(f"request: {request}")
            future = self.client.call_async(request)
            future.add_done_callback(self.get_ik_respone_callback)

        
        
    def check_all_process(self):       
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "KCF_Track"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "ALM_KCF_Tracker_Node"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "ALM_KCF_Tracker"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop"], check=False, timeout=2
            )
            '''subprocess.run(
                ["wmctrl", "-F", "-c", "KCF_follow"], check=False, timeout=2
            )'''
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_sort"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_apritag"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_remove_higher"], check=False, timeout=2
            )

            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove_color"], check=False, timeout=2
            )
            subprocess.run(["wmctrl", "-F", "-c", "color_remove_higher"], check=False, timeout=2)

            subprocess.run(["wmctrl", "-F", "-c", "follow_line"], check=False, timeout=2)
            subprocess.run(["wmctrl", "-F", "-c", "grasp_desktop_garbage_sort"], check=False, timeout=2)
            subprocess.run(["wmctrl", "-F", "-c", "detect_img"], check=False, timeout=2)
            subprocess.run(["wmctrl", "-F", "-c", "garbage_detect"], check=False, timeout=2)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "KCF_track"])
            subprocess.run(["pkill", "-f", "ALM_KCF_Tracker_Node"])
            subprocess.run(["pkill", "-f", "ALM_KCF_Tracker"])
            subprocess.run(["pkill", "-f", "grasp_desktop"])
            #subprocess.run(["pkill", "-f", "KCF_follow"])
            subprocess.run(["pkill", "-f", "apriltag_sort"])
            subprocess.run(["pkill", "-f", "grasp_desktop_apritag"])
            subprocess.run(["pkill", "-f", "grasp_desktop_remove"])
            subprocess.run(["pkill", "-f", "apriltag_remove_higher"])
            subprocess.run(["pkill", "-f", "grasp_desktop_remove_color"])
            subprocess.run(["pkill", "-f", "color_remove_higher"])
            subprocess.run(["pkill", "-f", "follow_line"])
            subprocess.run(["pkill", "-f", "grasp_desktop_garbage_sort"])
            subprocess.run(["pkill", "-f", "detect_img"])
            subprocess.run(["pkill", "-f", "garbage_detect"])




    # 核心程序，解析动作列表并执行  # Core program, parse and execute action list
    def execute_callback(self, goal_handle):
        """
        动作执行回调函数：分3种情况：  # Action execution callback function: divided into 3 cases:
        1. 动作列表为空  # 1. Empty action list
        2. 动作列表长度为1  # 2. Action list length is 1
        3. 动作列表长度大于1  # 3. Action list length is greater than 1
        文字交互模式下，不进行语音合成和播放  # In text interaction mode, no voice synthesis or playback is performed
        """

        if self.is_recording:
            goal_handle.succeed()
            result = Rot.Result()
            result.success = True
            return result

        feedback_msg = Rot.Feedback()
        actions = goal_handle.request.actions
        self.action_runing = True
        self.get_logger().info(f'"len(actions)": {len(actions)}')
        self.get_logger().info(f'"actions": {actions}')
        if not actions:  # 动作列表为空  # If the action list is empty
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio(self.tts_out_path, feedback=True)
            else:
                self.action_status_pub("response_done")

        elif len(actions) == 1:  # 动作列表长度为1  # If the action list length is 1

            action = actions[0]
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio(self.tts_out_path)

            match = re.match(r"(\w+)\((.*)\)", action)
            action_name, args_str = match.groups()
            if not hasattr(self, action_name):
                self.get_logger().warning(
                    f"action_service: {action} is invalid action,skip execution"
                )
                self.action_status_pub(
                    "failure_execute_action_function_not_exists"
                )  # Robot feedback: action function does not exist, cannot execute

            else:
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []
                method = getattr(self, action_name)
                method(*args)

            if self.interrupt_flag:
                self.interrupt_flag = False
        else:  # 动作列表长度大于1,使能组合模式  # If the action list length is greater than 1, enable combination mode

            self.combination_mode = True
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio_async(self.tts_out_path)

            for action in actions:
                self.get_logger().info(f'"action": {action}')
                if self.interrupt_flag:
                    break
                match = re.match(r"(\w+)\((.*)\)", action)
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []

                if not hasattr(self, action_name):
                    self.get_logger().warning(
                        f"action_service: {action} is invalid action，skip execution"  # action_service: {action} is an invalid action, skip execution
                    )
                    self.action_status_pub(
                        "failure_execute_action_function_not_exists"
                    )  # Robot feedback: action function does not exist, cannot execute
                else:
                    method = getattr(self, action_name)
                    method(*args)
                    feedback_msg.status = f"action service execute  {action}  successed"

            if not self.interrupt_flag:
                self.action_status_pub(
                    "multiple_done", actions=actions
                )  # Robot feedback: execution of {actions} completed
            self.combination_mode = (
                False  # 重置组合模式标志位  # Reset combination mode flag
            )
        self.action_runing = False  # 重置运行标志位  # Reset running flag
        self.interrupt_flag = False
        goal_handle.succeed()
        result = Rot.Result()
        result.success = True       
        return result

    def finish_dialogue(self):  # 发布AI模型结束当前流程标志
        self.first_record = True  
        self.is_recording = False  # 重置录音标志位  # Reset recording flag
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,2000)
        time.sleep(2.0)
        self.action_status_pub("finish")  # 结束当前任务

    def finishtask(self):
        """
        空操作,不反馈消息，用于结束反馈
        """
        #self.arm_move_flag = False
        return

    def kill_process_tree(pid):
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)

            # 先终止所有子进程
            for child in children:
                try:
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass

            # 等待子进程终止
            gone, alive = psutil.wait_procs(children, timeout=3)

            # 强制杀死仍然存活的进程
            for p in alive:
                try:
                    p.kill()
                except psutil.NoSuchProcess:
                    pass

            # 最后终止父进程
            try:
                parent.terminate()
                parent.wait(timeout=3)
            except psutil.TimeoutExpired:
                parent.kill()
            except psutil.NoSuchProcess:
                pass

        except psutil.NoSuchProcess:
            pass

    def play_audio(self, file_path: str, feedback: Bool = False) -> None:
        """
        同步方式播放音频函数The function for playing audio in synchronous mode
        """
        if self.is_recording:
            return
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            if self.stop_event.is_set() or self.is_recording:
                pygame.mixer.music.stop()
                self.stop_event.clear()  # 清除事件
                return
            pygame.time.Clock().tick(10)
        if feedback:
            self.action_status_pub("response_done")

    def play_audio_async(self, file_path: str, feedback: Bool = False) -> None:
        """
        异步方式播放音频函数The function for playing audio in asynchronous mode
        """
        if self.is_recording:
            return

        def target():
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                if self.stop_event.is_set() or self.is_recording:
                    pygame.mixer.music.stop()
                    self.stop_event.clear()  # 清除事件
                    return
                pygame.time.Clock().tick(5)
            if feedback:
                self.action_status_pub("response_done")

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()

    def save_single_image(self):
        """
        保存一张图片 / Save a single image
        """
        self.IS_SAVING=True
        time.sleep(0.5)
        if self.image_msg is None:
            return
        try:
            # 将ROS图像消息转换为OpenCV图像 / Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
            # 保存图片 / Save the image
            cv2.imwrite(self.image_save_path, cv_image)
            display_thread = threading.Thread(target=self.display_saved_image)
            display_thread.start()

        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")  # 保存图像时出错...
        self.IS_SAVING=False

    def display_saved_image(self):
        """
        显示已保存的图片4秒后关闭窗口 / Display the saved image for 4 seconds before closing the window
        """
        try:
            img = cv2.imread(self.image_save_path)
            if img is not None:
                cv2.imshow("Saved Image", img)
                cv2.waitKey(4000)  # 等待4秒 / Wait for 4 seconds
                cv2.destroyAllWindows()
            else:
                self.get_logger().error(
                    "Failed to load saved image for display."
                )  # 加载保存的图像以供显示失败...
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")  # 显示图像时出错...

    def image_callback(self, msg):  # 图像回调函数 / Image callback function
        if not self.IS_SAVING:
            self.image_msg = msg
        else:
            self.get_logger().error("The image is being saved and no new information will be accepted")


    def light_on(self,color):
        color = color.strip("'\"")  # 去掉单引号和双引号
        self.get_logger().info("Trun on the RGB Light.")
        if color == "red":
            self.get_logger().info("Trun on the Red Light.")
            Arm.Arm_RGB_set(50, 0, 0) #RGB亮红灯
        elif color == "green":
            self.get_logger().info("Trun on the Green Light.")
            Arm.Arm_RGB_set(0, 50, 0) #RGB亮绿灯
        elif color == "blue":
            self.get_logger().info("Trun on the Blue Light.")
            Arm.Arm_RGB_set(0, 0, 50) #RGB亮蓝灯   
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("light_on_done")
        
    def light_off(self):
        Arm.Arm_RGB_set(0, 0, 0)    
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("light_off_done")

    def beep_on(self):
        Arm.Arm_Buzzer_On()    
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("beep_on_done")

    def beep_off(self):
        Arm.Arm_Buzzer_Off()    
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("beep_off_done")

    def adjust_joint(self,joint_id,angle):
        Arm.Arm_serial_servo_write(int(joint_id), int(angle), 1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("adjust_joint_done")

    def gripper_open(self):
        Arm.Arm_serial_servo_write(6, 0, 1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("gripper_open_done")        


    def gripper_close(self):
        Arm.Arm_serial_servo_write(6, 180, 1000)
        time.sleep(2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("gripper_close_done")

    def set_pose(self):
        #Arm.Arm_serial_servo_write(6, 180, 1000)
        Arm.Arm_Buzzer_On()
        time.sleep(0.5)
        Arm.Arm_Buzzer_Off()
        status = input("Press y or Y when you finish to set the pose.")
        if status == 'y' or status == 'Y':
            self.action_status_pub("set_pose_done")

    def color_back_to_orin(self):
        self.back_list = True
        Arm.Arm_Buzzer_On()
        time.sleep(0.5)
        Arm.Arm_Buzzer_Off()
        for key in reversed(self.cur_pose.keys()):
            if key == 'red':
                self.action_status_pub("send_red_back_to_orin")
            elif  key == 'green':
                self.action_status_pub("send_green_back_to_orin")
            elif key == 'blue':
                self.action_status_pub("send_blue_back_to_orin")
            elif key == 'yellow':
                self.action_status_pub("send_yellow_back_to_orin") 
            while not self.return_done:
                time.sleep(0.1)
            self.return_done = False
        self.action_status_pub("return_to_orin_done")
        self.back_list = False

    def check_remove(self,color):
        check_color = color.strip("'\"")
        if check_color in self.cur_rm_pose:
            self.action_status_pub("color is in the rm_list")
        else:
            self.action_status_pub("color is not in the rm_list")  


    def grasp_from_rm_list(self,color):
        tar_color = color.strip("'\"")
        tar_joints = self.cur_rm_pose.get(tar_color)
        Arm.Arm_serial_servo_write6(tar_joints[0], tar_joints[1], tar_joints[2], tar_joints[3], tar_joints[4], tar_joints[5],2000)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write(6, 140, 1000)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(2.0)
        self.action_status_pub("grasp_from_rm_list_done")
        
    def arm_move(self,Dir,Dist):
        self.arm_move_flag = True
        cur_joints = [0.0,0.0,0.0,0.0,0.0,0.0]
        for i in range(1,7):
            cur_joints[i-1] = Arm.Arm_serial_servo_read(i)
            self.get_logger().info(f"cur_joints[i-1]: {cur_joints[i-1]}")
            if cur_joints[i-1] == None:
                cur_joints[i-1] = 0
                #self.get_logger().info('Servo Reading...')
        self.cur_joints = cur_joints
        Dir = Dir.strip("'\"")  # 去掉单引号和双引号
        self.Dir = Dir
        Dist = int(Dist)
        self.get_logger().info(f"Dir: {Dir}")
        self.get_logger().info(f"Dist: {Dist}")
        self.get_logger().info(f"cur_joints: {cur_joints}")
        self.cur_joints[0] = float(self.cur_joints[0])
        self.cur_joints[1] = float(self.cur_joints[1])
        self.cur_joints[2] = float(self.cur_joints[2])
        self.cur_joints[3] = float(self.cur_joints[3])
        self.cur_joints[4] = float(self.cur_joints[4])
        self.cur_joints[5] = float(self.cur_joints[5])
        self.get_current_end_pos()
        time.sleep(2)
        self.get_logger().info(f"CurEndPos: {self.CurEndPos}")
        self.move(Dir,Dist)
        time.sleep(2.0)

    def move(self,Dir,Dist):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')        
        request = Kinemarics.Request()
        if Dir == 'left':
            request.tar_x = self.CurEndPos[0] - Dist*0.01 
        elif Dir == 'right':
            request.tar_x = self.CurEndPos[0] + Dist*0.01 
        else:
            request.tar_x = self.CurEndPos[0]
        if Dir == 'forward':
            request.tar_y = self.CurEndPos[1]  + Dist*0.01
        elif Dir == 'backwards':
            request.tar_y = self.CurEndPos[1]  - Dist*0.01
        else:
            request.tar_y = self.CurEndPos[1] 
        if Dir == 'up':
            request.tar_z = self.CurEndPos[2] + Dist*0.01 
        elif Dir == 'down':
            request.tar_z = self.CurEndPos[2] - Dist*0.01 
        else:
            request.tar_z = self.CurEndPos[2]
        request.kin_name = "ik"
        request.roll = self.CurEndPos[3] 
        request.pitch = self.CurEndPos[4]
        request.yaw = math.atan(request.tar_x/request.tar_y)
        self.get_logger().info(f"request: {request}")
        future = self.client.call_async(request)
        future.add_done_callback(self.get_ik_respone_callback)
        
    def get_ik_respone_callback(self, future):        
        try:
            response = future.result()
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = int(response.joint1) #response.joint1
            joints[1] = int(response.joint2) 
            joints[2] = int(response.joint3) 
            if response.joint4>90:
                joints[3] = 90 
            else:
                joints[3] = int(response.joint4)
            joints[4] = 90 
            joints[5] = 30
            '''for i in range(0,6):
                if joints[i] <0:
                    joints[i]=0'''
            time.sleep(1.5)
            if self.Dir == 'left' or self.Dir == 'right':
                joints[1] = self.cur_joints[1]
                joints[2] = self.cur_joints[2]
                joints[3] = self.cur_joints[3]
            elif  self.Dir == 'down' or self.Dir == 'up' or self.Dir == 'forward' or self.Dir == 'backwards':
                joints[0] = self.cur_joints[0]             
            self.get_logger().info(f"joints: {joints}")
            if self.return_flag == True:
                Arm.Arm_serial_servo_write6(joints[0], joints[1], joints[2], joints[3], 90, self.grasp_joint,2000)
                time.sleep(2.0)
                Arm.Arm_serial_servo_write(6, 0, 2000)
                time.sleep(2.0)
                
            else:
                Arm.Arm_serial_servo_write6(joints[0], joints[1], joints[2], joints[3], 90, self.grasp_joint,2000)
                time.sleep(2.0)
                for i in range(6):
                    if joints[i] <0:
                        joints[i] = 0
                self.cur_joints = joints
            if self.arm_move_flag == False:
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                time.sleep(2.0)
                if self.back_list == False:
                    self.action_status_pub("return_to_orin_done")
                self.return_done = True
            else:
                #time.sleep(2.0)
                self.get_logger().info("Moving.")
                self.get_logger().info(f"self.combination_mode: {self.combination_mode}")
                self.get_logger().info(f"self.interrupt_flag: {self.interrupt_flag}")
                if not self.combination_mode and not self.interrupt_flag:
                    self.get_logger().info("Move done.")
                    self.action_status_pub("arm_move_done")
            time.sleep(2.0)
                    
                    
            
        except Exception:
           pass    



    def check_close_arm_stack(self):
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/Stack_Grap_Move")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)
        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)

    
            
    def arm_stack(self,color):
        tar_color = color.strip("'\"")
        if tar_color == 'red':
            Arm.Arm_serial_servo_write6(117, 19, 66, 56, 90,self.grasp_joint,2000)
            time.sleep(2.0)
        elif tar_color == 'green':
            Arm.Arm_serial_servo_write6(136, 66, 20, 29, 90,self.grasp_joint,2000)
            time.sleep(2.0)
        elif tar_color == 'blue':
            Arm.Arm_serial_servo_write6(44, 66, 20, 28, 90,self.grasp_joint,2000)
            time.sleep(2.0)
        elif tar_color == 'yellow':
            Arm.Arm_serial_servo_write6(65, 22, 64, 56, 90,self.grasp_joint,2000)
            time.sleep(2.0)
        Arm.Arm_serial_servo_write(6, 30, 1500)
        time.sleep(1.5)
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(2.0)
        self.action_status_pub("arm_stack_done", color=color)
        if self.interrupt_flag:
            time.sleep(0.5)
            Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)



    def check_close_point_to(self):
        
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/ALM_Point_To")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/LargeModel_ws/install/largemodel_arm/lib/largemodel_arm/ALM_KCF_Tracker")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)     

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)
        
        


    def point_to(self, x1, y1, x2, y2):
        cmd1 = "ros2 run largemodel_arm ALM_Point_To"
        cmd2 = "ros2 run largemodel_arm ALM_KCF_Tracker"
        # cmd3 = "ros2 run --prefix 'gdb -ex run --args' M3Pro_KCF ALM_KCF_Tracker_Node"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=ALM_KCF_Tracker",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        time.sleep(5.0) #等待ALM_KCF_Tracker_Node启动完成
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        time.sleep(2.0)
        if self.stack_flag == True:
            self.get_logger().info('Publish the stack_step topic...') 
            step_ = Int16()
            step_.data = self.step
            self.step_pub.publish(step_)
            self.step = self.step + 1
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))

        while not self.grasp_obj_future.done():
            if self.interrupt_flag:
                self.check_close_grasp_obj()
                return
            time.sleep(0.1)

        result = self.grasp_obj_future.result()
        self.check_close_point_to()
        if not self.interrupt_flag:
            if result.data == "grasp_obj_done":
                self.action_status_pub("point_to_done", x1=x1, y1=y1, x2=x2, y2=y2)
            else:
                self.action_status_pub("point_to_failed", x1=x1, y1=y1, x2=x2, y2=y2)
        
        self.grasp_obj_future = Future()  # 复位Future对象
        if self.interrupt_flag:
            time.sleep(0.5)
            Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)



    def check_garbage_sort_all(self):
        self.get_logger().info('Checking...') 
        pid_num = self.get_pid_by_command("/home/jetson/dofbot_pro_ws/src/dofbot_pro_yolov11/dofbot_pro_yolov11/yolov11_ALM.py")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)

        pid_num = self.get_pid_by_command("/home/jetson/dofbot_pro_ws/install/dofbot_pro_yolov11/lib/dofbot_pro_yolov11/yolov11_sortation")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM)   

        pid_num = self.get_pid_by_command("/home/jetson/dofbot_pro_ws/install/dofbot_pro_yolov11/lib/dofbot_pro_yolov11/msgToimg")
        self.get_logger().info(f"pid_num:{pid_num}")
        time.sleep(2.0)
        os.kill(pid_num[0], signal.SIGTERM) 

        sleep_pids = self.get_ss_plus_bash_pids()
        self.get_logger().info(f"sleep_pids:{sleep_pids}")
        os.kill(sleep_pids[-1], signal.SIGKILL)
        time.sleep(2.0)
        os.kill(sleep_pids[-2], signal.SIGKILL)
        time.sleep(2.0)
        #os.kill(sleep_pids[-3], signal.SIGKILL)
        #time.sleep(2.0)

        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
                           
    def garbage_sort_all(self):
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(1.0)
        cmd1 = "ros2 run dofbot_pro_yolov11 yolov11_sortation"
        cmd2 = "ros2 run dofbot_pro_yolov11 msgToimg"
        cmd3 = "python /home/jetson/dofbot_pro_ws/src/dofbot_pro_yolov11/dofbot_pro_yolov11/yolov11_ALM.py"
        
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_garbage_sort",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=detect_img",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=garbage_detect",
                "--",
                "bash",
                "-c",
                f"{cmd3}; exec bash",
            ]
        )

        while not self.garbage_sort_future.done():
            if self.interrupt_flag:
                self.check_garbage_sort()
                Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
                return
            time.sleep(0.1)

        result = self.garbage_sort_future.result()
        self.check_garbage_sort_all()
        if not self.interrupt_flag:
            if result.data == "garbage_sort_all_done":
                self.action_status_pub(
                    "garbage_sort_all_done"
                )

        
        self.garbage_sort_future = Future()  # 复位Future对象


    def grasp_from_down_list(self,color):
        tar_color = color.strip("'\"")
        tar_joints = self.cur_down_pose.get(tar_color)
        Arm.Arm_serial_servo_write6(tar_joints[0], tar_joints[1], tar_joints[2], tar_joints[3], tar_joints[4], 0,2000)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write(6, 140, 1000)
        time.sleep(2.0)
        Arm.Arm_serial_servo_write6(90,120,0,0,90,30,1000)
        time.sleep(2.0)
        self.action_status_pub("grasp_from_down_list_done", color=color)            




def main(args=None):
    rclpy.init(args=args)
    custom_action_server = CustomActionServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(custom_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        custom_action_server.stop()
        pass
    finally:
        custom_action_server.stop()
        custom_action_server.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



