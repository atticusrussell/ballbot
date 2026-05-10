#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
import qrcode
from PIL import Image
from pathlib import Path

class QRCodeNode(Node):
    def __init__(self):
        super().__init__('qr_code_node')
        self.declare_parameter('file_path', '/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config')
        self.declare_parameter('logo_filename', 'yahboom_logo.png')
        self.declare_parameter('output_filename', 'yahboom_logo_qr.jpg')

        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value
        self.logo_filename = self.get_parameter('logo_filename').get_parameter_value().string_value
        self.output_filename = self.get_parameter('output_filename').get_parameter_value().string_value

        self.qr_text = input("Enter the content to be printed by the QR code scan:")

        self.create_qrcode(self.qr_text, os.path.join(self.file_path, self.output_filename), os.path.join(self.file_path, self.logo_filename))

    def add_logo(self, img, logo_path):
        # Add logo, open logo image
        icon = Image.open(logo_path)
        img_w, img_h = img.size

        # Set the size of the logo
        factor = 6
        size_w = int(img_w / factor)
        size_h = int(img_h / factor)
        icon_w, icon_h = icon.size

        if icon_w > size_w:
            icon_w = size_w
        if icon_h > size_h:
            icon_h = size_h

        # Resize the logo
        icon = icon.resize((icon_w, icon_h), Image.Resampling.LANCZOS)

        # Center the logo
        w = int((img_w - icon_w) / 2)
        h = int((img_h - icon_h) / 2)

        # Paste the logo
        img.paste(icon, (w, h), mask=None)

        return img

    def create_qrcode(self, data, file_name, logo_path):
        """
        version: Integer from 1 to 40, controls the size of the QR code.
        error_correction: Controls the error correction function. Can be one of the following:
            ERROR_CORRECT_L: About 7% or fewer errors can be corrected.
            ERROR_CORRECT_M (default): About 15% or fewer errors can be corrected.
            ERROR_CORRECT_H: About 30% or fewer errors can be corrected.
        box_size: Controls the number of pixels in each box of the QR code.
        border: Controls the number of boxes for the border (the default is 4).
        """
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4
        )
        # Add data to the QR code
        qr.add_data(data)
        qr.make(fit=True)
        img = qr.make_image(fill_color="green", back_color="white")

        # Add logo if file exists
        if Path(logo_path).is_file():
            img = self.add_logo(img, logo_path)

        # Save and show the image
        img.save(file_name)
        img.show()

        return img

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()