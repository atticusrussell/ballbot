import os
import json
import rclpy
from rclpy.node import Node
from interfaces.action import Rot
from std_msgs.msg import String
from utils import large_model_interface
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from utils.promot import get_prompt, get_map_mapping
import time
import re
import functools


def measure_execution_time(func):
    """
    装饰器：测量函数执行时间并使用 ROS 日志打印结果
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        start_time = time.time()
        result = func(self, *args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        
        # 使用 ROS 日志系统记录执行时间
        if hasattr(self, 'get_logger'):
            self.get_logger().info(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        else:
            print(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        return result
    return wrapper

class LargeModelService(Node):
    def __init__(self):
        super().__init__("LargeModelService")

        self.init_param_config()  # 初始化参数配置 / Initialize parameter configuration
        self.init_largemodel()  # 初始化大模型 / Initialize large model
        self.init_ros_comunication()  # 初始化ROS通信 / Initialize ROS communication
        self.init_language()  # 初始化语言/Initialize language

        self.get_logger().info(
            "LargeModelService node Initialization completed..."
        )  # 打印日志 / Print log

    def init_largemodel(self):
        # 创建模型接口客户端 / Create model interface client
        self.model_client = large_model_interface.model_interface()
        self.new_order_cycle = True  # 新指令周期标志 / New order cycle flag
        if self.regional_setting == "China":  # 如果是中国地区
            #self.model_client.init_Multimodal()  # 初始化执行层模型，决策层模型无需初始化 / Initialize execution layer model, decision layer model does not need initialization
            self.model_client.init_dify_client()
        elif self.regional_setting == "international":  # 如果是国际地区
            self.model_client.init_dify_client()
        else:
            while True:
                self.get_logger().info()(
                    'Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".'
                )
                time.sleep(1)

    def init_param_config(self):
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        self.video_save_path = "/home/jetson/record_video.mp4"
        # 参数声明 / Parameter declaration

        self.declare_parameter("language", "zh")
        self.declare_parameter("regional_setting", "China")
        self.declare_parameter("text_chat_mode", False)
        # 获取参数服务器参数 / Get parameters from the parameter server
        self.language = (
            self.get_parameter("language").get_parameter_value().string_value
        )
        self.regional_setting = (
            self.get_parameter("regional_setting").get_parameter_value().string_value
        )
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )

        self.conversation_id = None  # 会话id
        self.map_mapping = get_map_mapping()  # 加载地图映射关系

    def init_language(self):
        self.language_dict = {
            "zh": "中文",
            "en": "English",
        }
        language_list = ["zh", "en"]
        if self.language not in language_list:
            while True:
                self.get_logger().info(
                    "The language setting is incorrect. Please check the action_service'' language setting in the yahboom.yaml file"
                )
                self.get_logger().info(self.language)
                time.sleep(1)
        self.prompt_dict = {  #
            "zh": {  # 中文 / Chinese
                "prompt_1": "用户：{prompt},决策层AI规划:{execute_instructions}",
                "prompt_2": "机器人反馈:执行seewhat()完成",
                "prompt_3": "决策层AI规划:{execute_instructions}",
            },
            "en": {  # 英文 / English
                "prompt_1": "user:{prompt},Decision making AI planning:{execute_instructions}",
                "prompt_2": "Robot feedback: Execute seewhat() completed",
                "prompt_3": "Decision making AI planning:{execute_instructions}",
            },
        }

    def init_ros_comunication(self):
        # 创建执行动作状态订阅者 / Create action status subscriber
        self.actionstatus_sub = self.create_subscription(
            String, "actionstatus", self.actionstatus_callback, 1
        )
        # 创建动作客户端，连接到 'action_service' / Create action client, connect to 'action_service'
        self._action_client = ActionClient(self, Rot, "action_service")
        # asr话题订阅者 / ASR topic subscriber
        self.asrsub = self.create_subscription(String, "asr", self.asr_callback, 1)
        # 创建seehat订阅者 / Create seewhat subscriber
        self.seewhat_sub = self.create_subscription(
            String, "seewhat_handle", self.seewhat_callback, 1
        )
        self.video_handle_sub = self.create_subscription(
            String, "video_handle", self.video_understanding_callback, 1
        )
        # 创建执行动作状态发布者 / Create action status publisher
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 1)
        # 创建文字交互发布者 / Create text interaction publisher
        self.text_pub = self.create_publisher(String, "text_response", 1)

    def seewhat_callback(self, msg):
        if msg.data == "seewhat":
            if (
                self.regional_setting == "China"
            ):  # 在线模型推理方式：决策层推理+执行层监督 / Online model inference method: Decision layer reasoning + Execution layer supervision
                #self.dual_large_model_mode(type="image")
                self.dual_large_model_international_model(type="image")
            else:
                self.dual_large_model_international_model(type="image")

    def video_understanding_callback(self, msg):
        if msg.data == "video_handle":
            if (
                self.regional_setting == "China"
            ):  # 在线模型推理方式：决策层推理+执行层监督 / Online model inference method: Decision layer reasoning + Execution layer supervision
                self.get_logger().info(
                    "continue->dual_large_model_mode"
                )
                #self.dual_large_model_mode(type="video")
                self.dual_large_model_international_model(type="video")
            else:
                self.dual_large_model_international_model(type="image")

                

    def asr_callback(self, msg):
        if (
            self.regional_setting == "China"
        ):  # 在线模型推理方式：决策层推理+执行层监督 / Online model inference method: Decision layer reasoning + Execution layer supervision
            #self.dual_large_model_mode(type="text", prompt=msg.data)
            self.dual_large_model_international_model(type="text", prompt=msg.data)
        else:
            self.dual_large_model_international_model(type="text", prompt=msg.data)

    def actionstatus_callback(self, msg):
        if (
            msg.data == "finish"
        ):  # 如果收到的是finish则表示当前指令执行完成，开启新的指令执行周期 / If "finish" is received, it means the current instruction has been executed and a new instruction cycle begins
            self.new_order_cycle = True
            self.get_logger().info(
                "The current instruction cycle has ended"
            )  # 当前指令周期已结束...
        else:  # 向指令执行层大模型反馈动作执行结果 / Feedback action execution results to the large model in the command execution layer
            if self.regional_setting == "China":
                #self.dual_large_model_mode(type="text", prompt=msg.data)
                self.dual_large_model_international_model(type="text", prompt=msg.data)
            else:
                self.dual_large_model_international_model(type="text", prompt=msg.data)

    # @measure_execution_time
    def dual_large_model_mode(self, type, prompt=""):
        """
        此函数实现了双模型推理模式，即先由文本生成模型进行任务规划，然后由多模态大模型生成动作列表
        This function implements the dual model inference mode, where the text generation model first plans the task, and then the multimodal large model generates the action list.
        """
        if (
            self.new_order_cycle
        ):  # 判断是否是新任务周期 / Determine if it is a new task cycle
            # 判断上一轮对话指令是否完成如果完成就清空历史上下文，开启新的上下文 / Determine if the previous round of dialogue instructions are completed. If completed, clear the historical context and start a new context
            self.model_client.init_Multimodal_history(
                get_prompt()
            )  # 初始化执行层上下文历史 / Initialize execution layer context history
            execute_instructions = self.model_client.TaskDecision(
                prompt
            )  # 调用决策层大模型进行任务规划 / Call the decision layer large model for task planning

            if not execute_instructions == "error":

                prompt_desidon = (
                    self.prompt_dict[self.language]
                    .get("prompt_3")
                    .format(execute_instructions=execute_instructions[1])
                )  # 翻译成对应语言的prompt /translate into the corresponding language prompt
                if self.text_chat_mode:
                    msg = String(data=prompt_desidon)
                    self.text_pub.publish(msg)
                else:
                    self.get_logger().info(prompt_desidon)  # 即将执行的任务：...

                prompt_desidon = (
                    self.prompt_dict[self.language]
                    .get("prompt_1")
                    .format(prompt=prompt, execute_instructions=execute_instructions[1])
                )  # 翻译成对应语言的prompt /translate into the corresponding language prompt

                self.instruction_process(
                    type="text",
                    prompt=prompt_desidon,
                )  # 传递决策层模型规划好的执行步骤给执行层模型 / Pass the planned execution steps from the decision layer model to the execution layer model

                self.new_order_cycle = (
                    False  # 重置指令周期标志位 / Reset the instruction cycle flag
                )
            else:
                self.get_logger().info(
                    "The model service is abnormal. Check the large model account or configuration options"
                )  # 模型推理失败，请检查模型配额和账户是否正常！！！
        else:
            self.get_logger().info(
                    "continue->instruction_process"
                )  
            self.instruction_process(
                prompt, type
            )  # 调用执行层大模型生成成动作列表并执行 / Call the execution layer large model to generate an action list and execute

    def instruction_process(self, prompt, type, conversation_id=None):
        """
        根据输入信息的类型（文字/图片），构建不同的请求体进行推理，并返回结果）
        Based on the type of input information (text/image), construct different request bodies for inference and return the result.
        """
        prompt_seewhat = self.prompt_dict[self.language].get("prompt_2")
        '''if self.regional_setting == "China":  # 国内版
            if type == "text":
                raw_content = self.model_client.multimodalinfer(prompt)
            elif type == "image":
                raw_content = self.model_client.multimodalinfer(
                    prompt_seewhat, image_path=self.image_save_path
                )
                self.get_logger().info(f'"prompt_seewhat": {prompt_seewhat}')
            elif type == "video":
                self.get_logger().info(
                    "continue->multimodalinfer"
                )
                raw_content = self.model_client.multimodalinfer(
                    prompt_seewhat, video_path=self.video_save_path
                )
            json_str = self.extract_json_content(raw_content)

        elif self.regional_setting == "international":  # 国际版
            if type == "text":
                result = self.model_client.TaskExecution(
                    input=prompt,
                    map_mapping=self.map_mapping,
                    language=self.language_dict[self.language],
                    conversation_id=conversation_id,
                )
                if result[0]:
                    json_str = self.extract_json_content(result[1])
                    self.conversation_id = result[2]
                else:
                    self.get_logger().info(f"ERROR:{result[1]}")
            elif type == "image":
                result = self.model_client.TaskExecution(
                    input=prompt_seewhat,
                    map_mapping=self.map_mapping,
                    language=self.language_dict[self.language],
                    image_path=self.image_save_path,
                    conversation_id=conversation_id,
                )
                if result[0]:
                    json_str = self.extract_json_content(result[1])
                    self.conversation_id = result[2]
                else:
                    self.get_logger().info(f"ERROR:{result[1]}")
            elif type == "video":
                result = self.model_client.TaskExecution(
                    input=prompt_seewhat,
                    map_mapping=self.map_mapping,
                    language=self.language_dict[self.language],
                    image_path=self.video_save_path,
                    conversation_id=conversation_id,
                )
                if result[0]:
                    json_str = self.extract_json_content(result[1])
                    self.conversation_id = result[2]
                else:
                    self.get_logger().info(f"ERROR:{result[1]}")'''
        if self.regional_setting == "international" or self.regional_setting == "China": 
            if type == "text":
                result = self.model_client.TaskExecution(
                    input=prompt,
                    map_mapping=self.map_mapping,
                    language=self.language_dict[self.language],
                    conversation_id=conversation_id,
                )
                if result[0]:
                    json_str = self.extract_json_content(result[1])
                    self.conversation_id = result[2]
                else:
                    self.get_logger().info(f"ERROR:{result[1]}")
            elif type == "image":
                self.get_logger().info('get an image...')
                result = self.model_client.TaskExecution(
                    input=prompt_seewhat,
                    map_mapping=self.map_mapping,
                    language=self.language_dict[self.language],
                    image_path=self.image_save_path,
                    conversation_id=conversation_id,
                )
                if result[0]:
                    json_str = self.extract_json_content(result[1])
                    self.conversation_id = result[2]
                else:
                    self.get_logger().info(f"ERROR:{result[1]}")
            elif type == "video":
                result = self.model_client.TaskExecution(
                    input=prompt_seewhat,
                    map_mapping=self.map_mapping,
                    language=self.language_dict[self.language],
                    video_path=self.video_save_path,
                    conversation_id=conversation_id,
                )
                if result[0]:
                    json_str = self.extract_json_content(result[1])
                    self.conversation_id = result[2]
                else:
                    self.get_logger().info(f"ERROR:{result[1]}")

        if json_str is not None:
            self.get_logger().info(
                f'"json_str": {json_str}'
            )
            # 解析JSON字符串,分离"action"、"response"字段 / Parse JSON string, separate "action" and "response" fields
            action_plan_json = json.loads(json_str)
            action_list = action_plan_json.get("action", [])
            llm_response = action_plan_json.get("response", "")
        else:
            self.get_logger().info(
                f"LargeScaleModel return: {json_str},The format was unexpected. The output format of the AI model at the execution layer did not meet the requirements"
            )
            return

        if self.text_chat_mode:
            msg = String(data=f'"action": {action_list}, "response": {llm_response}')
            self.text_pub.publish(msg)
        else:
            self.get_logger().info(
                f'"action": {action_list}, "response": {llm_response}'
            )

        self.send_action_service(
            action_list, llm_response
        )  # 异步发送动作列表、回复内容给ActionServer / Asynchronously send action list and response content to ActionServer

    def dual_large_model_international_model(self, type, prompt=""):
        """
        此函数适用于国际版双模型推理模式,使用dify作为中间件
        /this function is suitable for international model inference mode, using dify as the middleware
        """
        if (
            self.new_order_cycle
        ):  # 判断是否是新任务周期 / Determine if it is a new task cycle
            self.conversation_id = None
            result = self.model_client.TaskDecision(prompt)

            if result[0]:
                prompt_desidon = (
                    self.prompt_dict[self.language]
                    .get("prompt_3")
                    .format(execute_instructions=result[1])
                )  # 翻译成对应语言的prompt /translate into the corresponding language prompt
                if self.text_chat_mode:  # 文字交互模式 / Text interaction mode
                    msg = String(data=prompt_desidon)
                    self.text_pub.publish(msg)
                else:  # 语音交互模式 / Voice interaction mode
                    self.get_logger().info(prompt_desidon)
                prompt_desion = (
                    self.prompt_dict[self.language]
                    .get("prompt_1")
                    .format(prompt=prompt, execute_instructions=result[1])
                )
                self.instruction_process(type="text", prompt=prompt_desion)

                self.new_order_cycle = (
                    False  # 重置指令周期标志位 / Reset the instruction cycle flag
                )
            else:
                self.get_logger().info(
                    "The model service is abnormal. Check the large model account or configuration options"
                )  # 模型推理失败，请检查模型配额和账户是否正常！！！

        else:
            self.instruction_process(
                prompt, type, conversation_id=self.conversation_id
            )  # 调用执行层大模型生成成动作列表并执行 / Call the execution layer large model to generate an action list and execute

    def send_action_service(self, actions, text):
        goal_msg = Rot.Goal()  # 创建目标消息对象 / Create goal message object
        goal_msg.actions = actions  # 设置目标消息中的动作列表 / Set the action list in the goal message
        goal_msg.llm_response = text
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # 添加目标发送后的响应回调函数 / Add response callback function after sending the goal
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()  # 获取目标句柄 / Get goal handle
        if not goal_handle.accepted:
            self.get_logger().info(
                "action_client message: action service rejected action list"
            )  # 目标被拒绝...

    @staticmethod
    def extract_json_content(
        raw_content,
    ):  # 解析变量提取json / Extract JSON by parsing variables
        try:
            # 方法一：分割代码块 / Method 1: Split code blocks
            if "```json" in raw_content:
                # 分割代码块并取中间部分 / Split code blocks and take the middle part
                json_str = raw_content.split("```json")[1].split("```")[0].strip()
            elif "```" in raw_content:
                # 处理没有指定类型的代码块 / Handle code blocks without specified types
                json_str = raw_content.split("```")[1].strip()
            else:
                # 直接尝试解析 / Try parsing directly
                json_str = raw_content

            # 方法二：正则表达式提取（备用方案） / Method 2: Regular expression extraction (backup plan)
            if not json_str:

                match = re.search(r"\{.*\}", raw_content, re.DOTALL)
                if match:
                    json_str = match.group()

            return json_str

        except Exception as e:
            return None



def main(args=None):
    rclpy.init(args=args)
    model_service = LargeModelService()
    rclpy.spin(model_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
