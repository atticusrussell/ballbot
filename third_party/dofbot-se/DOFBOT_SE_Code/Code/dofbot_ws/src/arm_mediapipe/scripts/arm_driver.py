#!/usr/bin/env python3
# encoding: utf-8
import rospy
from Arm_Lib import Arm_Device
from yahboomcar_msgs.msg import *
from std_msgs.msg import Bool
import time
Arm = Arm_Device()

joints = [90, 2.0, 60.0, 40.0, 90]
Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1000)
time.sleep(1)
print("init done")
def Armcallback(msg):
	if not isinstance(msg, ArmJoint): 
		print("----------")
		return
	arm_joint = ArmJoint()
	print("msg.joints: ",msg.joints)
	print("msg.joints: ",msg.run_time)
	if len(msg.joints) != 0:
		arm_joint.joints = joints
		for i in range(2):
			Arm.Arm_serial_servo_write6(msg.joints[0], msg.joints[1],msg.joints[2],msg.joints[3],msg.joints[4],msg.joints[5],time=msg.run_time)
			time.sleep(0.01)
	else:
		arm_joint.id = msg.id
		arm_joint.angle = msg.angle
		for i in range(2):
			Arm.Arm_serial_servo_write(msg.id, msg.angle, msg.run_time)
			time.sleep(0.01)
			
def Buzzercallback(msg):
	if not isinstance(msg, Bool): 
		print("----------")
		return
	if msg.data==True:
		print("beep on")	
		Arm.Arm_Buzzer_On()
	else:
		Arm.Arm_Buzzer_Off()
		print("beep off")	
			
sub_Arm = rospy.Subscriber("TargetAngle", ArmJoint, Armcallback, queue_size=1000)
sub_Buzzer = rospy.Subscriber("Buzzer", Bool, Buzzercallback,queue_size=1000)

if __name__ == '__main__':
	Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1000)
	rospy.init_node("arm_driver_node",anonymous=True)
	rospy.spin()
