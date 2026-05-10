import os
import serial
import time
class kws_mic:
    def __init__(self, port,kwsquence, baudrate=115200, timeout=1):

        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.running = False
        self.kws_queue = kwsquence
    def open(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.ser.is_open:
                print(f"serial {self.port} open")
                self.running = True
        except Exception as e:
            print(f"open serial fail: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.running = False
            self.ser.close()
            print(f"serial {self.port} close")

    def send_data(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data.encode('utf-8')) 
            except Exception as e:
                print(f"send fial: {e}")

    def receive_data(self):
        step = 1
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    data = self.ser.read() 
                    if data:
                        dealdata = bytearray(data)[0]
                        if dealdata == 0xAA and step ==1:
                            step = 2
                        elif dealdata == 0x55 and step ==2:
                            step = 3
                        elif (dealdata == 0x01 or dealdata == 0x02 or dealdata == 0x03 or dealdata == 0x04 or dealdata == 0x05 or dealdata == 0x06) and step ==3:
                            step = 4
                        elif dealdata == 0x00 and step ==4:
                            step = 5
                        elif dealdata == 0xFB and step ==5:
                            # print("kws detected")
                            self.kws_queue.put("resonse_1")
                            step = 1

                except Exception as e:
                    print(f"recvice fail: {e}")
            time.sleep(0.1)


