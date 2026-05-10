#!/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import threading
import cv2 as cv
import time
from time import sleep
import smbus
from dofbot_utils.robot_controller import Robot_Controller


i2c_speech_addr = 0x30   #语音播报模块地址 Voice broadcast module address
speech_date_head = 0xfd


EncodingFormat_Type = {
                        'GB2312':0x00,
                        'GBK':0X01,
                        'BIG5':0x02,
                        'UNICODE':0x03
                        }

ChipStatus_Type = {
                    'ChipStatus_InitSuccessful':0x4A,#初始化成功回传 Initialization successful return
                    'ChipStatus_CorrectCommand':0x41,#收到正确的命令帧回传 Receive the correct command frame back
                    'ChipStatus_ErrorCommand':0x45,#收到不能识别命令帧回传 Received an unrecognized command frame and returned it
                    'ChipStatus_Busy':0x4E,#芯片忙碌状态回传 Chip Busy Status Return
                    'ChipStatus_Idle':0x4F #芯片空闲状态回传 Chip idle state return
                }

Style_Type = {
                'Style_Single':0,#为 0，一字一顿的风格 is 0, word-by-word style
                'Style_Continue':1#为 1，正常合成 1, normal synthesis
                }#合成风格设置 [f?]

Language_Type = {
                'Language_Auto':0,#为 0，自动判断语种
                'Language_Chinese':1,#为 1，阿拉伯数字、度量单位、特殊符号等合成为中文
                'Language_English':2#为 1，阿拉伯数字、度量单位、特殊符号等合成为中文
                }#合成语种设置 [g?]

Articulation_Type = {
                'Articulation_Auto':0,#为 0，自动判断单词发音方式
                'Articulation_Letter':1,#为 1，字母发音方式
                'Articulation_Word':2#为 2，单词发音方式
                }#设置单词的发音方式 [h?]

Spell_Type = {
                'Spell_Disable':0,#为 0，不识别汉语拼音
                'Spell_Enable':1#为 1，将“拼音＋1 位数字（声调）”识别为汉语拼音，例如： hao3
                }#设置对汉语拼音的识别 [i?]

Reader_Type = {
                'Reader_XiaoYan':3,#为 3，设置发音人为小燕(女声, 推荐发音人)
                'Reader_XuJiu':51,#为 51，设置发音人为许久(男声, 推荐发音人)
                'Reader_XuDuo':52,#为 52，设置发音人为许多(男声)
                'Reader_XiaoPing':53,#为 53，设置发音人为小萍(女声
                'Reader_DonaldDuck':54,#为 54，设置发音人为唐老鸭(效果器)
                'Reader_XuXiaoBao':55#为 55，设置发音人为许小宝(女童声)
                }#选择发音人 [m?]

NumberHandle_Type = {
                'NumberHandle_Auto':0,#为 0，自动判断
                'NumberHandle_Number':1,#为 1，数字作号码处理
                'NumberHandle_Value':2#为 2，数字作数值处理
                }#设置数字处理策略 [n?]

ZeroPronunciation_Type = {
                'ZeroPronunciation_Zero':0,#为 0，读成“zero
                'ZeroPronunciation_O':1#为 1，读成“欧”音
                }#数字“0”在读 作英文、号码时 的读法 [o?]

NamePronunciation_Type = {
                'NamePronunciation_Auto':0,#为 0，自动判断姓氏读音
                'NamePronunciation_Constraint':1#为 1，强制使用姓氏读音规则
                }#设置姓名读音 策略 [r?]

OnePronunciation_Type = {
                'OnePronunciation_Yao':0,#为 0，合成号码“1”时读成幺
                'OnePronunciation_Yi':1#为 1，合成号码“1”时读成一
                }#设置号码中“1”的读法 [y?]

Rhythm_Type = {
                'Rhythm_Diasble':0,#为 0，“ *”和“#”读出符号
                'Rhythm_Enable':1#为 1，处理成韵律，“*”用于断词，“#”用于停顿
                }#是否使用韵律 标记“*”和“#” [z?]    


class speech_color_sorting:
    def __init__(self):
        '''
        设置初始化参数
        '''
        self.image = None
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        self.robot = Robot_Controller()
        self.bus = smbus.SMBus(self.arm.get_i2c_bus_num())
        # 夹爪加紧角度
        self.grap_joint = self.robot.get_gripper_value(1)
        # 夹取位置
        self.joints = self.robot.P_CENTER
        self.joints[5] = self.robot.get_gripper_value(0)
        self.SetReader(Reader_Type["Reader_XiaoPing"])#选择播音人晓萍
        self.SetVolume(8)

    def get_Sqaure(self, color_name, hsv_lu):
        '''
        颜色识别
        '''
        (lowerb, upperb) = hsv_lu
        # 设置识别区域
        point_Xmin=50   #200
        point_Xmax=600  #440
        point_Ymin=80   #200
        point_Ymax=480  #480
        # 画矩形框
        # cv.rectangle(self.image, (point_Xmin, point_Ymin), (point_Xmax, point_Ymax),(105,105,105), 2)
        # 复制原始图像,避免处理过程中干扰
        img = self.image.copy()
        mask = img[point_Ymin:point_Ymax, point_Xmin:point_Xmax]

        # 将图像转换为HSV。
        HSV_img = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        img = cv.inRange(HSV_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            # boundingRect函数计算边框值，x，y是坐标值，w，h是矩形的宽和高
            x, y, w, h = cv.boundingRect(cnt)
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积范围
            if area > 1000:
                # 中心坐标
                x_w_ = float(x + w / 2)
                y_h_ = float(y + h / 2)
                # 在img图像画出矩形，(x, y), (x + w, y + h)是矩形坐标，(0, 255, 0)设置通道颜色，2是设置线条粗度
                cv.rectangle(self.image, (x + point_Xmin, y + point_Ymin), (x + w + point_Xmin, y + h + point_Ymin), (0, 255, 0), 2)
                # 绘制矩形中心
                cv.circle(self.image, (int(x_w_ + point_Xmin), int(y_h_ + point_Ymin)), 5, (0, 0, 255), -1)
                # # 在图片中绘制结果
                cv.putText(self.image, color_name, (int(x + point_Xmin-15), int(y + point_Ymin-15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                
                # cv.rectangle(self.image, (x , y ), (x + w , y + h ), (0, 255, 0), 2)
                # cv.circle(self.image, (int(x_w_ ), int(y_h_ )), 5, (0, 0, 255), -1)
                # cv.putText(self.image, color_name, (int(x -15), int(y -15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return (x_w_, y_h_)

    def Sorting_grap(self, img, color_hsv):
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480))
        # 获取识别的结果
        msg = {}
        if self.status == 'waiting':
            # 遍历颜色通道,获取能够识别的结果
            for key, value in color_hsv.items():
                point = self.get_Sqaure(key, value)
                if point != None: msg["name"] = key
            if len(msg) == 1:
                self.num += 1
                # 每当连续识别10次并且运动状态为waiting的情况下,执行抓取任务
                if self.num % 10 == 0 and self.status == 'waiting':
                    self.status = "Runing"
                    self.arm.Arm_Buzzer_On(1)
                    sleep(0.5)
                    # 开启抓取线程
                    threading.Thread(target=self.sorting_run, args=(msg['name'],)).start()
                    self.num = 0
        return self.image

    def sorting_move(self, joints_target):
        '''
        移动过程
        '''
        joints_up = self.robot.P_TOP
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # Release clamping jaws 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # Move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(self.joints, 1000)
        sleep(1)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # rotate 旋转
        self.arm.Arm_serial_servo_write(1, joints_target[0], 1000)
        sleep(1)
        # Move to corresponding position 移动至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.8)
        # raise  抬起
        self.arm.Arm_serial_servo_write(2, 90, 1000)
        sleep(1)
        # return to center 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)
        # move to initial position 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_MAP, 1000)
        sleep(1.5)

    def sorting_run(self, name):
        '''
        机械臂移动函数
        '''
        if name == "red" :
            self.Speech_text("识别到红色",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # print("red")
            # 物体放置位姿
            # 移动
            self.sorting_move(self.robot.P_RED)
            self.Speech_text("放置完成",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # 抓取完毕
            self.status = 'waiting'
        if name == "blue":
            self.Speech_text("识别到蓝色",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # print("blue")
            self.sorting_move(self.robot.P_BLUE)
            self.Speech_text("放置完成",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # 抓取完毕
            self.status = 'waiting'
        if name == "green" :
            self.Speech_text("识别到绿色",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # print("green")
            self.sorting_move(self.robot.P_GREEN)
            self.Speech_text("放置完成",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # 抓取完毕
            self.status = 'waiting'
        if name == "yellow" :
            self.Speech_text("识别到黄色",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # print("yellow")
            self.sorting_move(self.robot.P_YELLOW)
            self.Speech_text("放置完成",EncodingFormat_Type["GB2312"])
            while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                time.sleep(0.1)  
            # 抓取完毕
            self.status = 'waiting'


    def I2C_WriteBytes(self, str_):
        global i2c_speech_addr
        for ch in str_:
            try:
                self.bus.write_byte(i2c_speech_addr,ch)
                time.sleep(0.01)
            except:
                print("write I2C error")


    def Speech_text(self, str_,encoding_format):
        str_ = str_.encode('gb2312')   
        size = len(str_)+2
        DataHead = speech_date_head
        Length_HH = size>>8
        Length_LL = size & 0x00ff
        Commond = 0x01
        EncodingFormat = encoding_format

        Date_Pack = [DataHead,Length_HH,Length_LL,Commond,EncodingFormat]

        self.I2C_WriteBytes(Date_Pack)

        self.I2C_WriteBytes(str_)

    def SetBase(self, str_):
        str_ = str_.encode('gb2312')   
        size = len(str_)+2

        DataHead = speech_date_head
        Length_HH = size>>8
        Length_LL = size & 0x00ff
        Commond = 0x01
        EncodingFormat = 0x00

        Date_Pack = [DataHead,Length_HH,Length_LL,Commond,EncodingFormat]

        self.I2C_WriteBytes(Date_Pack)

        self.I2C_WriteBytes(str_)

    def TextCtrl(self, ch,num):
        if num != -1:
            str_T = '[' + ch + str(num) + ']'
            self.SetBase(str_T)
        else:
            str_T = '[' + ch + ']'
            self.SetBase(str_T)


    def GetChipStatus(self):
        global i2c_speech_addr
        AskState = [0xfd,0x00,0x01,0x21]
        try:
            self.I2C_WriteBytes(AskState)
            time.sleep(0.05)
        except:
            print("I2CRead_Write error")


        try:
            Read_result = self.bus.read_byte(i2c_speech_addr)
            return Read_result
        except:
            print("I2CRead error")

    def SetStyle(self, num):
        self.TextCtrl('f',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)   


    def SetLanguage(self, num):
        self.TextCtrl('g',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    def SetArticulation(self, num):
        self.TextCtrl('h',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetSpell(self, num):
        self.TextCtrl('i',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetReader(self, num):
        self.TextCtrl('m',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetNumberHandle(self, num):
        self.TextCtrl('n',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)



    def SetZeroPronunciation(self, num):
        self.TextCtrl('o',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)




    def SetNamePronunciation(self, num):
        self.TextCtrl('r',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #设置语速 [s?] ? 为语速值，取值：0～10
    def SetSpeed(self, speed):
        self.TextCtrl('s',speed)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    #设置语调 [t?] ? 为语调值，取值：0～10
    def SetIntonation(self, intonation):
        self.TextCtrl('t',intonation)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #设置音量 [v?] ? 为音量值，取值：0～10
    def SetVolume(self, volume):
        self.TextCtrl('v',volume)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)



    def SetOnePronunciation(self, num):
        self.TextCtrl('y',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)


    def SetRhythm(self, num):
        self.TextCtrl('z',num)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

    #恢复默认的合成参数 [d] 所有设置（除发音人设置、语种设置外）恢复为默认值
    def SetRestoreDefault(self):
        self.TextCtrl('d',-1)
        while self.GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
            time.sleep(0.002)

