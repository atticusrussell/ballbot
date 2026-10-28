#!/usr/bin/env python3
"""Find the forward/reverse dead zone. Drives each speed forward then back, so the
robot ends roughly where it started. Needs ~0.5 m clear in front and behind.

Uses encoder odometry (/odom/unfiltered): we only need to know if the wheels turned.
"""
import rclpy, time, statistics
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEEDS  = [0.01, 0.02, 0.03, 0.04, 0.05, 0.07, 0.10]
SPIN_UP = 1.0   # s, discarded
MEASURE = 2.0   # s, averaged
PAUSE   = 1.5   # s, stopped between runs
BE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

class Cal(Node):
    def __init__(s):
        super().__init__('fwd_deadzone')
        s.pub = s.create_publisher(Twist, '/cmd_vel_joy', 10)
        s.vx, s.rec = [], False
        s.create_subscription(Odometry, '/odom/unfiltered',
            lambda m: s.vx.append(m.twist.twist.linear.x) if s.rec else None, BE)

    def drive(s, vx, secs, recording):
        t = Twist(); t.linear.x = vx
        s.rec = recording
        end = time.time() + secs
        while time.time() < end:
            s.pub.publish(t)
            rclpy.spin_once(s, timeout_sec=0.05)
        s.rec = False

    def run(s, vx):
        s.vx = []
        s.drive(vx, SPIN_UP, False)
        s.drive(vx, MEASURE, True)
        v = list(s.vx)
        s.drive(0.0, PAUSE, False)
        return statistics.median(v) if len(v) >= 10 else None

def main():
    rclpy.init(); n = Cal()
    print(f"{'cmd vx':>8} {'odom vx':>9} {'achieved/cmd':>13}")
    try:
        for vx in SPEEDS:
            for sign in (1, -1):
                v = n.run(sign*vx)
                if v is None:
                    print(f"{sign*vx:8.2f}   -- no odom data --"); continue
                print(f"{sign*vx:8.2f} {v:9.3f} {abs(v)/vx:13.3f}")
    finally:
        n.drive(0.0, 1.0, False)
    print("\nDead zone ends where achieved/cmd reaches ~1. Set min_speed_xy just above it.")
    rclpy.shutdown()

main()
