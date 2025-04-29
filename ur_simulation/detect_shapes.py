#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
import threading

def mark_nongray_area(image, min_val = 0, max_val = 255, gray_factor=0.1):
    gray_tolerance = (max_val-min_val)*gray_factor
    color_dev = np.max(image, axis=2) - np.min(image, axis=2)
    img_dims = np.shape(image)[:2]

    mask = np.where(color_dev <= gray_tolerance, np.zeros(img_dims), np.ones(img_dims))
    mask = mask.astype(np.float32)
    return mask

class DetectShapes(Node):
    def __init__(self):
        super().__init__('detect_shapes')
        self.mask_publisher = self.create_publisher(Image, '/camera/mask_image', 10)
        self.image_subscriber = self.create_subscription(Image, '/camera/image', self.convert,
                                                         qos_profile=qos_profile_sensor_data)

    def convert(self, msg: Image):
        cv_bridge = CvBridge()
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask_image = mark_nongray_area(image)

        mask_msg = cv_bridge.cv2_to_imgmsg(mask_image)
        self.mask_publisher.publish(mask_msg)

    def run(self):
        while rclpy.ok():
           pass

def main(args=None):
    rclpy.init(args=args)

    vp_node = DetectShapes()

    thread = threading.Thread(target=rclpy.spin, args=(vp_node,), daemon=True)
    thread.start()
    vp_node.run()

    vp_node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()