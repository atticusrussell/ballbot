#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import time
import cv2 as cv
import numpy as np

class OpenPoseDetectionNode(Node):
    def __init__(self):
        super().__init__('openpose_detection_node')

        # Set up camera capture
        self.capture = cv.VideoCapture('/dev/video0')
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

        # Load the COCO class names and DNN model for object detection
        with open('/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/object_detection_coco.txt', 'r') as f: 
            self.class_names = f.read().split('\n')
        self.COLORS = np.random.uniform(0, 255, size=(len(self.class_names), 3))
        self.model = cv.dnn.readNet(model='/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/frozen_inference_graph.pb', config='/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/ssd_mobilenet_v2_coco.txt', framework='TensorFlow')

        # Set up OpenPose model
        self.BODY_PARTS = {
            "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
            "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
            "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
            "LEye": 15, "REar": 16, "LEar": 17, "Background": 18
        }
        self.POSE_PAIRS = [
            ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
            ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
            ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
            ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
            ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]
        ]
        self.net = cv.dnn.readNetFromTensorflow("/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/graph_opt.pb")

        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer for periodic callback to process frames

    def timer_callback(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from camera.")
            return

        # frame = self.openpose(frame)
        frame = self.target_detection(frame)

        # Show the processed frame directly in a window
        cv.imshow('OpenPose Detection', frame)
        
        # Wait for 'q' key press to stop the loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting the application...")
            self.destroy_node()
            cv.destroyAllWindows()

    def target_detection(self, image):
        image_height, image_width, _ = image.shape
        # Create blob from image
        blob = cv.dnn.blobFromImage(image=image, size=(300, 300), mean=(104, 117, 123), swapRB=True)
        self.model.setInput(blob)
        output = self.model.forward()

        for detection in output[0, 0, :, :]:
            confidence = detection[2]
            if confidence > 0.4:
                class_id = detection[1]
                class_name = self.class_names[int(class_id) - 1]
                color = self.COLORS[int(class_id)]
                box_x = detection[3] * image_width
                box_y = detection[4] * image_height
                box_width = detection[5] * image_width
                box_height = detection[6] * image_height
                cv.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), color, thickness=2)
                cv.putText(image, class_name, (int(box_x), int(box_y - 5)), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        return image

    def openpose(self, frame):
        frameHeight, frameWidth = frame.shape[:2]
        self.net.setInput(cv.dnn.blobFromImage(frame, 1.0, (368, 368), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        out = self.net.forward()
        out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements
        points = []
        for i in range(len(self.BODY_PARTS)):
            heatMap = out[0, i, :, :]
            _, conf, _, point = cv.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            points.append((int(x), int(y)) if conf > 0.2 else None)
        for pair in self.POSE_PAIRS:
            partFrom = pair[0]
            partTo = pair[1]
            idFrom = self.BODY_PARTS[partFrom]
            idTo = self.BODY_PARTS[partTo]
            if points[idFrom] and points[idTo]:
                cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
                cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
                cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
        return frame

def main(args=None):
    rclpy.init(args=args)
    node = OpenPoseDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.capture.release()
        cv.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
