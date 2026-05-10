#coding=utf-8
import socket
import os
import time
from Arm_Lib import Arm_Device



global g_init
g_init = False
Arm = Arm_Device()


# 获取本机IP
def getLocalip():
    ip = os.popen("/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0 : ip.find('\n')]
    if(ip == ''):
        #读取WLAN的IP
        ip = os.popen("/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0 : ip.find('\n')]
        if(ip == ''):
            ip = 'x.x.x.x'
    return ip


#协议解析部分
def Analysis(socket, cmd):
    print(cmd)
    #    $20111222333444555666#
    try :
        check = cmd[1:3]
        if check == '20' and len(cmd) == 22:
            s1_angle = int(cmd[3:6])
            s2_angle = int(cmd[6:9])
            s3_angle = int(cmd[9:12])
            s4_angle = int(cmd[12:15])
            s5_angle = int(cmd[15:18])
            s6_angle = int(cmd[18:21])

            Arm.Arm_serial_servo_write6(s1_angle, s2_angle, s3_angle, s4_angle, s5_angle, s6_angle, 500)
        else:
            print("cmd data error! continue...")
    except:
        print("cmd data error! continue...")

#socket TCP通信建立
def start_tcp_server(ip, port):
    global g_init, g_socket
    g_init = True
    print('start_tcp_server')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.listen(5)
    while True:
        conn, address = sock.accept()
        g_socket = conn
        print("client connected:" + str(address))
        while True:
            cmd = g_socket.recv(1024).decode(encoding="utf-8") 
            if not cmd:
                print("Client Disconnected!")
                break
            # print('   [-]cmd:', cmd, len(cmd))
            index1 = cmd.find("$")
            index2 = cmd.find("#")
            if index1 >= 0 and index2 > index1:
                Analysis(g_socket, cmd[index1:index2+1])



#关闭socket
def waitClose():
    global g_init, g_socket
    g_init = False
    g_socket.close()


if __name__ == '__main__':
    try:
        port = 6100
        if g_init == False:
            while(True):
                ip = getLocalip()
                print("%s:%d" % (ip, port))
                if(ip == "x.x.x.x"):
                    continue
                if(ip != "x.x.x.x"):
                    break
        start_tcp_server(ip, port)
    except KeyboardInterrupt:
        waitClose()
        print(" Program closed! ")
        pass
