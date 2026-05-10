#coding=utf-8
import socket
import os
import time
from Arm_Lib import Arm_Device



global g_sock
Arm = Arm_Device()


# socket客户端
def connect_tcp_server(ip, port):
    global g_sock
    print("Connecting server...")
    g_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    g_sock.connect((ip, port))
    print("Connected!")
    time.sleep(1)
    Arm.Arm_serial_set_torque(0)
    last_angle = [0, 0, 0, 0, 0, 0]
    while True:
        angle = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            id = i + 1
            angle[i] = Arm.Arm_serial_servo_read(id)
            if angle[i] == None:
                time.sleep(.001)
                angle[i] = Arm.Arm_serial_servo_read(id)
                if angle[i] == None:
                    angle[i] = last_angle[i]

            last_angle[i] = angle[i]
            time.sleep(.001)

        pos1str = "%03d" % angle[0]
        pos2str = "%03d" % angle[1]
        pos3str = "%03d" % angle[2]
        pos4str = "%03d" % angle[3]
        pos5str = "%03d" % angle[4]
        pos6str = "%03d" % angle[5]
        data = "$20"+pos1str+pos2str+pos3str+pos4str+pos5str+pos6str+"#"
        print(data)
        b_data = bytes(data, encoding = "utf8")
        g_sock.send(b_data)
        time.sleep(.1)



#关闭socket
def waitClose(sock):
    sock.close()
    Arm.Arm_serial_set_torque(1)


if __name__ == '__main__':
    #根据服务器的IP地址修改以下参数
    ip = '192.168.2.100'
    port = 6100
    try:
        connect_tcp_server(ip, port)
    except KeyboardInterrupt:
        waitClose(g_sock)
        print(" Program closed! ")
        pass
