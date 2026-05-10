# coding: utf-8
'''
@Copyright (C): 2010-2019, Shenzhen Yahboom Tech
@Author: Malloy.Yuan
@Date: 2019-07-30 20:34:09
@LastEditors: Malloy.Yuan
@LastEditTime: 2019-08-08 16:10:46
'''


# *****************************************************************#
#                      Incremental PID system                     #
# *****************************************************************#
class IncrementalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.PIDOutput = 0.0  # PID控制器输出       PID controller output
        self.SystemOutput = 0.0  # 系统输出值          System output value
        self.LastSystemOutput = 0.0  # 上次系统输出值       Last system output value

        self.Error = 0.0  # 输出值与输入值的偏差  Deviation between input value and output value
        self.LastError = 0.0
        self.LastLastError = 0.0

    # 设置PID控制器参数  Set PID controller parameters
    def SetStepSignal(self, StepSignal):
        self.Error = StepSignal - self.SystemOutput
        IncrementValue = self.Kp * (self.Error - self.LastError) + \
                         self.Ki * self.Error + \
                         self.Kd * (self.Error - 2 * self.LastError + self.LastLastError)

        self.PIDOutput += IncrementValue
        self.LastLastError = self.LastError
        self.LastError = self.Error
       
    # Set the first-order inertial link system, where inertiatime is the inertial time constant
    # 设置一阶惯性环节系统  其中InertiaTime为惯性时间常数
    def SetInertiaTime(self, InertiaTime, SampleTime):
        self.SystemOutput = (InertiaTime * self.LastSystemOutput + \
                             SampleTime * self.PIDOutput) / (SampleTime + InertiaTime)
        print(self.SystemOutput)
        self.LastSystemOutput = self.SystemOutput


# *****************************************************************#
#                      Position type PID system                    #
# *****************************************************************#
class PositionalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.SystemOutput = 0.0
        self.ResultValueBack = 0.0
        self.PidOutput = 0.0
        self.PIDErrADD = 0.0
        self.ErrBack = 0.0

    # Set PID controller parameters
    # 设置PID控制器参数
    def SetStepSignal(self, StepSignal):
        Err = StepSignal - self.SystemOutput
        KpWork = self.Kp * Err
        KiWork = self.Ki * self.PIDErrADD
        KdWork = self.Kd * (Err - self.ErrBack)
        self.PidOutput = KpWork + KiWork + KdWork
        self.PIDErrADD += Err
        self.ErrBack = Err

    # Set the first-order inertial link system, where inertiatime is the inertial time constant
    # 设置一阶惯性环节系统  其中InertiaTime为惯性时间常数
    def SetInertiaTime(self, InertiaTime, SampleTime):
        self.SystemOutput = (InertiaTime * self.ResultValueBack + \
                             SampleTime * self.PidOutput) / (SampleTime + InertiaTime)
        print(self.SystemOutput)
        self.ResultValueBack = self.SystemOutput
