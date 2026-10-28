#!/usr/bin/env python3
"""Open-loop yaw calibration for LR_WHEELS_DISTANCE.

Commands a steady angular.z, waits out the spin-up transient, then averages the
gyro over a steady window. The gyro is independent of LR_WHEELS_DISTANCE, so it is
a valid instrument for calibrating it. /odom/unfiltered is logged for comparison
only -- it is computed WITH the constant under test, so it cannot be ground truth.

Robot spins in place. Clear floor required.
"""
import rclpy, time, statistics, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

LR_CURRENT = 0.291          # firmware/config/lino_base_config.h:61
SPEEDS     = [0.1, 0.2, 0.3, 0.4]
SPIN_UP    = 2.0            # s, discarded
MEASURE    = 4.0            # s, averaged
PAUSE      = 2.5            # s, stopped between runs
BE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

class Cal(Node):
    def __init__(s):
        super().__init__('yaw_cal')
        s.pub = s.create_publisher(Twist, '/cmd_vel_joy', 10)
        s.gyro, s.odom, s.rec = [], [], False
        s.create_subscription(Imu, '/imu/data',
            lambda m: s.gyro.append(m.angular_velocity.z) if s.rec else None, BE)
        s.create_subscription(Odometry, '/odom/unfiltered',
            lambda m: s.odom.append(m.twist.twist.angular.z) if s.rec else None, BE)

    def drive(s, wz, secs, recording):
        t = Twist(); t.angular.z = wz
        s.rec = recording
        end = time.time() + secs
        while time.time() < end:
            s.pub.publish(t)
            rclpy.spin_once(s, timeout_sec=0.05)
        s.rec = False

    def run(s, wz):
        s.gyro, s.odom = [], []
        s.drive(wz, SPIN_UP, False)
        s.drive(wz, MEASURE, True)
        g, o = list(s.gyro), list(s.odom)
        s.drive(0.0, PAUSE, False)
        if len(g) < 10: return None
        return statistics.median(g), (statistics.median(o) if o else float('nan')), len(g)

def main():
    rclpy.init(); n = Cal()
    print(f"{'cmd w':>8} {'gyro w':>9} {'odom w':>9} {'achieved/cmd':>13} {'odom/gyro':>10} {'n':>5}")
    ratios = []
    try:
        for wz in SPEEDS:
            for sign in (1, -1):
                r = n.run(sign*wz)
                if not r:
                    print(f"{sign*wz:8.2f}   -- no gyro data --"); continue
                gm, om, cnt = r
                a_c = abs(gm)/wz
                ratios.append(a_c)
                print(f"{sign*wz:8.2f} {gm:9.3f} {om:9.3f} {a_c:13.3f} {abs(om/gm) if gm else 0:10.3f} {cnt:5d}")
    finally:
        n.drive(0.0, 1.0, False)
    if ratios:
        m = statistics.median(ratios)
        print(f"\nmedian achieved/commanded = {m:.3f}")
        spread = (max(ratios)-min(ratios))/m
        print(f"spread {min(ratios):.3f}..{max(ratios):.3f}  (+/-{spread/2*100:.1f}% of median)")
        print("  <5%: clean scale factor, one constant fixes it.  >15%: nonlinear, a constant won't.")
        print(f"\nLR_WHEELS_DISTANCE: {LR_CURRENT}  ->  {LR_CURRENT/m:.4f}")
        print("  Commanding w gives wheel speed w*b/2, so true rotation = w*b/b_eff.")
        print("  achieved/cmd = b/b_eff  =>  b_eff = b / ratio.  Over-turning shrinks the value.")
    rclpy.shutdown()

main()
