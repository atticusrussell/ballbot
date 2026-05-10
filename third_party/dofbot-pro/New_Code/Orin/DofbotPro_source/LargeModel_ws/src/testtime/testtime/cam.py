import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from queue import Queue
import time

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # 声明并获取参数，默认订阅话题为/image_raw
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # 创建图像消息订阅者
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            100  # QoS设置
        )
        
        # 初始化CvBridge用于ROS图像和OpenCV图像转换
        self.bridge = CvBridge()
        
        # 创建线程安全的队列存储图像
        self.image_queue = Queue(maxsize=60)  # 限制队列最大长度，防止内存溢出
        
        # 标记是否运行的标志
        self.running = True
        
        # 创建并启动显示线程
        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.daemon = True  # 当主线程退出时，该线程也会退出
        self.display_thread.start()
        
        self.get_logger().info(f"订阅相机话题: {image_topic}")
        self.get_logger().info("按 'q' 键退出程序")

    def image_callback(self, msg):
        """图像消息回调函数，将图像转换后放入队列"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 如果队列已满，移除最旧的图像
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()
                except:
                    pass
            
            # 将图像放入队列
            self.image_queue.put(cv_image)
            
        except Exception as e:
            self.get_logger().error(f"图像转换错误: {str(e)}")

    def display_images(self):
        """显示图像的线程函数，从队列中获取图像并显示"""
        # 帧率计算相关变量
        frame_count = 0
        start_time = time.time()
        fps = 0
        
        while self.running:
            try:
                # 从队列获取图像，超时时间0.1秒
                cv_image = self.image_queue.get(timeout=0.1)
                
                # 计算帧率
                frame_count += 1
                elapsed_time = time.time() - start_time
                if elapsed_time >= 1.0:  # 每秒更新一次帧率
                    fps = frame_count / elapsed_time
                    frame_count = 0
                    start_time = time.time()
                
                # 在图像上绘制帧率
                cv2.putText(
                    cv_image, 
                    f"FPS: {fps:.1f}", 
                    (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    1, 
                    (0, 255, 0), 
                    2
                )
                
                # 显示图像
                cv2.imshow('Camera View', cv_image)
                
                # 检查按键，按'q'退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break
                
                # 标记任务完成
                self.image_queue.task_done()
                
            except:
                # 队列为空时的超时处理，继续循环
                continue
        
        # 关闭窗口
        cv2.destroyAllWindows()

    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.running = False
        if self.display_thread.is_alive():
            self.display_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    camera_viewer = CameraViewer()
    
    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保节点正确销毁
        camera_viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
