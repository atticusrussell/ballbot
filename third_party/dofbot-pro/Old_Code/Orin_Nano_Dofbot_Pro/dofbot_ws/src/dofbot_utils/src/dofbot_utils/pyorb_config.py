# !/usr/bin/env python3
# coding: utf-8
import time
from pyorbbecsdk import *


class Pyorb_Config:
    def __init__(self, debug=False):
        self.debug = debug
        self.context = Context()
        self.device_list = self.context.query_devices()
        self.device = self.device_list.get_device_by_index(0)
        self.pipeline = Pipeline(self.device)
        self.device.get_device_info()
        self.get_color_mirror()

        self.DEF_COLOR_MIRROR = False
        self.DEF_COLOR_AUTO_EXPOSURE = True
        self.DEF_COLOR_EXPOSURE = 33
        self.DEF_COLOR_GAIN = 0
        self.DEF_COLOR_SHARPNESS = 3
        self.DEF_COLOR_CONTRAST = 50
        self.DEF_COLOR_BRIGHTNESS = 30

    def __del__(self):
        self.pipeline = None
        self.device = None
        self.context = None
        del self.pipeline
        del self.device
        del self.context
        
    # 读取设备信息
    def get_device_info(self):
        device_info = self.device.get_device_info()
        # print("device_info:", device_info)
        return device_info

    # 读取相机名称
    def get_device_name(self):
        device_info = self.device.get_device_info()
        device_name = device_info.get_name()
        # print("device_name:", device_name)
        return device_name

    # 打开颜色相机的自动曝光功能
    def turn_on_color_auto_exposure(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color auto exposure!")
            return
        self.device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)

    # 关闭颜色相机的自动曝光功能
    def turn_off_color_auto_exposure(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color auto exposure!")
            return
        self.device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, False)

    # 设置颜色相机的自动曝光功能开关
    def set_color_auto_exposure(self, enable):
        if enable:
            self.turn_on_color_auto_exposure()
        else:
            self.turn_off_color_auto_exposure()

    # 读取颜色相机的自动曝光状态
    def get_color_auto_exposure(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color auto exposure!")
            return
        curr_color_auto_exposure = self.device.get_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL)
        return curr_color_auto_exposure

    # 设置彩色相机镜像开关
    def set_color_mirror(self, enable):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_MIRROR_BOOL,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color mirror!")
            return
        if enable:
            self.device.set_bool_property(OBPropertyID.OB_PROP_COLOR_MIRROR_BOOL, True)
        else:
            self.device.set_bool_property(OBPropertyID.OB_PROP_COLOR_MIRROR_BOOL, False)

    # 读取彩色相机镜像状态
    def get_color_mirror(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_MIRROR_BOOL,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color mirror!")
            return
        curr_color_mirror = self.device.get_bool_property(OBPropertyID.OB_PROP_COLOR_MIRROR_BOOL)
        return curr_color_mirror

    ###################################################################################################
    ###################################################################################################
    ###################################################################################################

    # 设置颜色相机的曝光值
    def set_color_exposure_value(self, color_exposure):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_EXPOSURE_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color exposure!")
            return
        self.turn_off_color_auto_exposure()
        self.device.set_int_property(OBPropertyID.OB_PROP_COLOR_EXPOSURE_INT, int(color_exposure))

    # 读取颜色相机的曝光值
    def get_color_exposure_value(self):
        curr_color_exposure = self.device.get_int_property(OBPropertyID.OB_PROP_COLOR_EXPOSURE_INT)
        # print("curr_color_exposure:", curr_color_exposure)
        return curr_color_exposure

    # 设置彩色相机的增益值
    def set_color_gain_value(self, color_gain):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_GAIN_INT,
                                                    OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color gain!")
            return
        self.turn_off_color_auto_exposure()
        self.device.set_int_property(OBPropertyID.OB_PROP_COLOR_GAIN_INT, int(color_gain))

    # 读取彩色相机的增益值
    def get_color_gain_value(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_GAIN_INT,
                                                    OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color gain!")
            return
        curr_color_gain = self.device.get_int_property(OBPropertyID.OB_PROP_COLOR_GAIN_INT)
        # print("curr_color_gain:", curr_color_gain)
        return curr_color_gain

    # 设置颜色相机的锐度（清晰度）
    def set_color_sharpness_value(self, color_sharpness):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_SHARPNESS_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color sharpness!")
            return
        self.turn_off_color_auto_exposure()
        self.device.set_int_property(OBPropertyID.OB_PROP_COLOR_SHARPNESS_INT, int(color_sharpness))

    # 读取颜色相机的锐度（清晰度）
    def get_color_sharpness_value(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_SHARPNESS_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color sharpness!")
            return
        curr_color_sharpness = self.device.get_int_property(OBPropertyID.OB_PROP_COLOR_SHARPNESS_INT)
        return curr_color_sharpness

    # 设置颜色相机的对比度数值
    def set_color_contrast_value(self, color_contrast):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_CONTRAST_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color contrast!")
            return
        self.turn_off_color_auto_exposure()
        self.device.set_int_property(OBPropertyID.OB_PROP_COLOR_CONTRAST_INT, int(color_contrast))

    # 读取颜色相机的对比度数值
    def get_color_contrast_value(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_CONTRAST_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color contrast!")
            return
        curr_color_contrast = self.device.get_int_property(OBPropertyID.OB_PROP_COLOR_CONTRAST_INT)
        return curr_color_contrast

    # 设置颜色相机的亮度值
    def set_color_brightness_value(self, color_brightness):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_BRIGHTNESS_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color brightness!")
            return
        self.turn_off_color_auto_exposure()
        self.device.set_int_property(OBPropertyID.OB_PROP_COLOR_BRIGHTNESS_INT, int(color_brightness))

    # 读取颜色相机的亮度值
    def get_color_brightness_value(self):
        if not self.device.is_property_supported(OBPropertyID.OB_PROP_COLOR_BRIGHTNESS_INT,
                                                 OBPermissionType.PERMISSION_READ_WRITE):
            print("Current device not support color brightness!")
            return
        curr_color_brightness = self.device.get_int_property(OBPropertyID.OB_PROP_COLOR_BRIGHTNESS_INT)
        return curr_color_brightness


    # 读取颜色相机所有配置参数
    def read_all_color_config(self):
        color_mirror = self.get_color_mirror()
        print("color_mirror:", color_mirror)

        color_auto_exposure = self.get_color_auto_exposure()
        print("color_auto_exposure:", color_auto_exposure)

        color_exposure = self.get_color_exposure_value()
        print("color_exposure:", color_exposure)

        color_gain = self.get_color_gain_value()
        print("color_gain:", color_gain)

        color_sharpness = self.get_color_sharpness_value()
        print("color_sharpness:", color_sharpness)

        color_contrast = self.get_color_contrast_value()
        print("color_contrast:", color_contrast)

        color_brightness = self.get_color_brightness_value()
        print("color_brightness:", color_brightness)


    # 设置彩色相机默认参数
    def default_color_config(self):
        self.set_color_mirror(self.DEF_COLOR_MIRROR)
        time.sleep(.1)
        # self.set_color_exposure_value(self.DEF_COLOR_EXPOSURE)
        # time.sleep(.02)
        # self.set_color_gain_value(self.DEF_COLOR_GAIN)
        # time.sleep(.02)
        # self.set_color_sharpness_value(self.DEF_COLOR_SHARPNESS)
        # time.sleep(.02)
        # self.set_color_contrast_value(self.DEF_COLOR_CONTRAST)
        # time.sleep(.02)
        # self.set_color_brightness_value(self.DEF_COLOR_BRIGHTNESS)
        # time.sleep(.02)
        self.set_color_auto_exposure(self.DEF_COLOR_AUTO_EXPOSURE)
        time.sleep(.1)
