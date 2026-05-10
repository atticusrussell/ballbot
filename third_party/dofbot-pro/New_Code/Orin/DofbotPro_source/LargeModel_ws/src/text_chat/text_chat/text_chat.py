import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import time
import os
import signal
import select
def reset_terminal():
    os.system("stty sane")

class TextChatNode(Node):
    def __init__(self):
        super().__init__('text_chat_node')
        self.asr_publisher = self.create_publisher(String, 'asr', 10)
        self.response_subscriber = self.create_subscription(String, 'text_response', self.response_callback, 10)
        self.running = True
        self.response_received = threading.Event()
        self.latest_response = None
        self.animation_active = False

        self.animation_thread = threading.Thread(target=self.display_animation)
        self.animation_thread.daemon = True   
        self.animation_thread.start()     # 启动动画线程 
        self.input_thread = threading.Thread(target=self.handle_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def handle_user_input(self):
        while self.running and rclpy.ok():
            try:
                # 读取输入时显式指定编码容错处理
                user_input = input("user input: ").strip()
                
                # 尝试用UTF-8解码（兼容GNOME终端默认编码），无法解码则替换错误字符
                user_input = user_input.encode('utf-8', errors='replace').decode('utf-8', errors='replace')
                
                if not user_input:
                    continue

                self.asr_publisher.publish(String(data=user_input))

                self.first_response = True
                self.response_received.clear()
                self.animation_active = True

                if self.response_received.wait(timeout=10):
                    self.first_response = False

            except UnicodeDecodeError as e:
                # self.get_logger().error(f"输入编码错误，请使用UTF-8字符（如GNOME终端默认编码）：{e}")
                continue
            except EOFError:
                break
            except KeyboardInterrupt:
                self.running = False
                reset_terminal()
                break
            except Exception as e:
                self.get_logger().error(f"输入处理异常：{e}")
                continue


    def response_callback(self, msg):
        self.get_logger().info(msg.data)
        if self.first_response:
            self.response_received.set()
        self.animation_active = False
        sys.stdout.write('\r' + ' ' * 30 + '\r')  # 清除动画

    def display_animation(self):
        animation_chars = ['|', '/', '-', '\\']
        i = 0
        while self.running:  
            if self.animation_active:
                sys.stdout.write(f'\r okay😀, let me think for a moment... {animation_chars[i % len(animation_chars)]}')
                sys.stdout.flush()
                time.sleep(0.1)
                i += 1
            else:
                time.sleep(0.1)  

def signal_handler(sig, frame):
    reset_terminal()
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)  # 捕获 Ctrl+C 信号
    rclpy.init(args=args)
    node = TextChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False  
        node.input_thread.join(timeout=1)  
        node.animation_thread.join(timeout=1)  
        node.destroy_node()
        rclpy.shutdown()
        reset_terminal()  
if __name__ == '__main__':
    main()


