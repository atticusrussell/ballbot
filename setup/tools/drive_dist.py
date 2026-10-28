#!/usr/bin/env python3
"""Drive straight until wheel odometry says TARGET metres, then stop.
Tape-measure the real distance: real / TARGET is the linear odometry scale error.
Needs ~1.5 m clear ahead."""
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET = 0.5   # m, by odometry
SPEED  = 0.1   # m/s commanded

rclpy.init(); n = Node('drive_dist')
pub = n.create_publisher(Twist, '/cmd_vel_joy', 10)
pos = {}
n.create_subscription(Odometry, '/odom/unfiltered',
    lambda m: pos.update(p=(m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
while 'p' not in pos: rclpy.spin_once(n, timeout_sec=0.1)
x0, y0 = pos['p']; t = Twist(); t.linear.x = SPEED
try:
    while math.hypot(pos['p'][0]-x0, pos['p'][1]-y0) < TARGET:
        pub.publish(t); rclpy.spin_once(n, timeout_sec=0.05)
finally:
    for _ in range(10): pub.publish(Twist()); time.sleep(0.05)
time.sleep(1.0); rclpy.spin_once(n, timeout_sec=0.1)
print(f"odometry says {math.hypot(pos['p'][0]-x0, pos['p'][1]-y0):.3f} m. Tape-measure the real distance.")
