#!/usr/bin/env python
# coding:utf-8
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, Image
from laser_geometry import LaserProjection


class pt2brid_eye:
    def __init__(self):
        self.bridge = CvBridge()
        self.laserProj = LaserProjection()
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)  # 接收scan节点  Receiving scan Nodes
        self.image_pub = rospy.Publisher('/laserImage', Image, queue_size=1)

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        lidar = point_cloud2.read_points(cloud_out)
        points = np.array(list(lidar))
        img = self.pointcloud_to_laserImage(points)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img))
        img = cv.resize(img, (640, 480))
        cv.imshow("img", img)
        cv.waitKey(10)

    def pointcloud_to_laserImage(self, points):  # 鸟瞰图生成  Aerial view generated
        x_points = points[:, 0]
        y_points = points[:, 1]
        z_points = points[:, 2]
        f_filt = np.logical_and((x_points > -50), (x_points < 50))
        s_filt = np.logical_and((y_points > -50), (y_points < 50))
        filter = np.logical_and(f_filt, s_filt)
        indices = np.argwhere(filter)
        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]
        x_img = (-y_points * 80).astype(np.int32) + 500
        y_img = (-x_points * 80).astype(np.int32) + 500
        pixel_values = np.clip(z_points, -2, 2)
        pixel_values = ((pixel_values + 2) / 4.0) * 500
        img = np.zeros([1600, 1200], dtype=np.uint8)
        img[y_img, x_img] = pixel_values
        return img


if __name__ == '__main__':
    print("opencv: {}".format(cv.__version__))
    rospy.init_node('laser_to_Image', anonymous=False)
    pt2img = pt2brid_eye()
    rospy.spin()
