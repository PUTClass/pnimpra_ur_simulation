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

def mark_nongray_area():
    mask = None
    ### Write a custom function to extract binary mask based on colours.
    ### Each pixel consists of RGB elements. It might be assumed that
    ### all gray pixel components are either equal or they are within a narrow range.
    ### In the final mask: 1 - objects, 0 - background
    ### Hint: numpy arrays can be converted to float32 -> mask.astype(np.float32)
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

    ds_node = DetectShapes()

    thread = threading.Thread(target=rclpy.spin, args=(ds_node,), daemon=True)
    thread.start()
    ds_node.run()

    ds_node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()