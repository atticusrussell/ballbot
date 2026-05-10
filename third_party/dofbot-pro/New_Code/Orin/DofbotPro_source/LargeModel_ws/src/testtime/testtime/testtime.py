import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class LidarTimestampChecker(Node):
    def __init__(self):
        super().__init__('lidar_timestamp_checker')
        
        # 订阅激光雷达话题，默认是/scan，可根据实际情况修改
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan0',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        self.get_logger().info('Lidar timestamp checker node started')

    def listener_callback(self, msg):
        # 获取激光雷达数据的时间戳
        lidar_timestamp = msg.header.stamp
        
        # 获取当前ROS时间
        current_ros_time = self.get_clock().now().to_msg()
        
        # 计算时间差（秒）
        time_diff = (current_ros_time.sec - lidar_timestamp.sec) + \
                   (current_ros_time.nanosec - lidar_timestamp.nanosec) / 1e9
        
        # 打印结果
        self.get_logger().info(
            f'scan0雷达时间戳: {lidar_timestamp.sec}.{lidar_timestamp.nanosec:09d}\n'
            f'ROS时钟: {current_ros_time.sec}.{current_ros_time.nanosec:09d}\n'
            f'相差时间：: {time_diff:.6f} seconds'
        )

def main(args=None):
    rclpy.init(args=args)
    
    lidar_timestamp_checker = LidarTimestampChecker()
    
    try:
        rclpy.spin(lidar_timestamp_checker)
    except KeyboardInterrupt:
        lidar_timestamp_checker.get_logger().info('Node interrupted by user')
    finally:
        # 销毁节点
        lidar_timestamp_checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
