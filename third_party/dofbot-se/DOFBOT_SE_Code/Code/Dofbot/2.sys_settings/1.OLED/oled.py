#!/usr/bin/env python3
#coding=utf-8
# 20210913更新，增加热插拔功能。
import time
import os

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

while True:
    try:
        #pin configuration,on the OLED this pin isnt used
        RST = None

        # 128x32 display with hardware I2C:
        disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, i2c_bus=1, gpio=1)

        # Initialize library.
        disp.begin()

        # Clear display.
        disp.clear()
        disp.display()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        width = disp.width
        height = disp.height
        image = Image.new('1', (width, height))

        # Get drawing object to draw on image.
        draw = ImageDraw.Draw(image)

        # Draw a black filled box to clear the image.
        draw.rectangle((0,0,width,height), outline=0, fill=0)

        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        top = padding
        bottom = height-padding
        # Move left to right keeping track of the current x position for drawing shapes.
        x = 0

        # Load default font.
        font = ImageFont.load_default()
    except :
        time.sleep(1)
        continue
    break

#读取CPU占用率
def getCPULoadRate():
    f1 = os.popen("cat /proc/stat", 'r')
    stat1 = f1.readline()
    count = 10
    data_1 = []
    for i  in range (count):
        data_1.append(int(stat1.split(' ')[i+2]))
    total_1 = data_1[0]+data_1[1]+data_1[2]+data_1[3]+data_1[4]+data_1[5]+data_1[6]+data_1[7]+data_1[8]+data_1[9]
    idle_1 = data_1[3]

    time.sleep(1)

    f2 = os.popen("cat /proc/stat", 'r')
    stat2 = f2.readline()
    data_2 = []
    for i  in range (count):
        data_2.append(int(stat2.split(' ')[i+2]))
    total_2 = data_2[0]+data_2[1]+data_2[2]+data_2[3]+data_2[4]+data_2[5]+data_2[6]+data_2[7]+data_2[8]+data_2[9]
    idle_2 = data_2[3]

    total = int(total_2-total_1)
    idle = int(idle_2-idle_1)
    usage = int(total-idle)
    # print("idle:"+str(idle)+"  total:"+str(total))
    usageRate = int(float(usage  / total) * 100)
    str_CPU = "CPU:"+str(usageRate)+"%"
    print(str_CPU)
    return str_CPU

#读取系统时间
def getSystemTime():
    cmd = "date +%H:%M:%S"
    date_time = subprocess.check_output(cmd, shell = True )
    str_Time = str(date_time).lstrip('b\'')
    str_Time = str_Time.rstrip('\\n\'')
    # print(date_time)
    return str_Time


#读取空闲的内存 / 总内存
def getFreeRAM():
    cmd = "free -h | awk 'NR==2{printf \"RAM: %.1f/%.1fGB \", $7,$2}'"
    FreeRam = subprocess.check_output(cmd, shell = True )
    str_FreeRam = str(FreeRam).lstrip('b\'')
    str_FreeRam = str_FreeRam.rstrip('\'')
    return str_FreeRam

#读取空闲的TF卡空间 / TF卡总空间
def getFreeDisk():
    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk:%.1f/%.1fGB\", $4,$2}'"
    Disk = subprocess.check_output(cmd, shell = True )
    str_Disk = str(Disk).lstrip('b\'')
    str_Disk = str_Disk.rstrip('\'')
    return str_Disk

#读取当前IP地址
def getLocalIP():
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    str_IP = str(IP).lstrip('b\'')
    str_IP = str_IP.rstrip('\\n\'')
    # print(str_IP)
    return str_IP


def main():
    try:
        while True:
            # Draw a black filled box to clear the image.
            draw.rectangle((0,0,width,height), outline=0, fill=0)

            #读取系统信息
            str_CPU = getCPULoadRate()
            str_Time = getSystemTime()
            str_FreeRAM = getFreeRAM()
            str_Disk = getFreeDisk()
            str_IP = getLocalIP()

            #OLED加载显示缓存信息
            draw.text((x, top), str_CPU, font=font, fill=255)
            draw.text((x+50, top), str_Time, font=font, fill=255)
            draw.text((x, top+8), str_FreeRAM,  font=font, fill=255)
            draw.text((x, top+16), str_Disk,  font=font, fill=255)
            draw.text((x, top+24), "ip:" + str_IP,  font=font, fill=255)
            

            # Display image.
            disp.image(image)
            disp.display()
            # time.sleep(.5)
    except:
        pass



if __name__ == "__main__":
    # try :
    #     main()
    # except KeyboardInterrupt:
    #     print(" Program closed! ")
    #     pass
    
    try:
        while True:
            main()
            time.sleep(2)
            try:
                disp.begin()
            except:
                pass
    except KeyboardInterrupt:
        print(" Program closed! ")
        pass
