import os
import sys
import getopt
import socket


if __name__ == "__main__":
    host = '0.0.0.0'
    port = 8000
    addr = (host, port)
    udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpServer.bind(addr)
    print("Searching for devices...")
    while True:
        data, addr = udpServer.recvfrom(1024)
        print(data, addr)
        msg = str(data, encoding = 'utf-8')
        print(msg)
        if msg == "YAHBOOMARM_FIND":
            udpServer.sendto(bytes("DOFBOT-Pro", encoding='utf-8'), addr)
            print("send ok")