#!/usr/bin/env python3

import rclpy
import rclpy.clock
import rclpy.clock_type
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from .point_cloud2 import read_points_numpy, create_cloud, create_cloud_xyz32


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from .transformations import quaternion_from_euler, quaternion_multiply

from rclpy.clock import Clock
from rclpy.clock_type import ClockType

import threading

def find_closest_point(origin, points):
    relative_points = points - origin
    distances = np.linalg.norm(relative_points, axis=-1)
    min_id = np.argmin(distances)
    return points[min_id]

def apply_mask_to_points(points, mask):
    points = np.reshape(points, (-1, 3))
    mask = np.reshape(mask, (-1,))
    non_zero_idx = np.nonzero(mask)
    return points[non_zero_idx]

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class ClosestObjects(Node):
    def __init__(self):
        super().__init__('closest_objects')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.mask_subscriber = self.create_subscription(Image, '/camera/mask_image', self.object_mask_callback,
                                                         qos_profile=qos_profile_sensor_data)
        
        self.point_cloud_subscriber = self.create_subscription(PointCloud2, '/camera/points', self.point_cloud_callback,
                                                         qos_profile=qos_profile_sensor_data)
        
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/camera/object_points', 5)
        
        self.mask, self.pcl = None, None

    def object_mask_callback(self, msg: Image):
        cv_bridge = CvBridge()
        mask = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.mask = mask

    def point_cloud_callback(self, msg: PointCloud2):
        self.pcl = msg

    def get_transform(self, to_frame_id, from_frame_id="world"):
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_id,
                from_frame_id,
                rclpy.time.Time())
            return t
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_id} to {from_frame_id}: {ex}')
            return None
        
    def send_transform(self, point, to_frame_id, from_frame_id):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = from_frame_id
        t.child_frame_id = to_frame_id

        t.transform.translation.x = float(point[0])
        t.transform.translation.y = float(point[1])
        t.transform.translation.z = float(point[2])

        q_x = quaternion_from_euler(0.0,  0.0, np.pi/2)
        q_y = quaternion_from_euler(0.0, np.pi/2, 0.0)

        q = quaternion_multiply(q_x, q_y)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def publish_objects_pcl(self, points, frame_id):
        header = Header()
        header.frame_id = frame_id
        header.stamp = self.get_clock().now().to_msg()
        pc2 = create_cloud_xyz32(header, points)
        self.point_cloud_publisher.publish(pc2)
    

    def run(self):
        while rclpy.ok():
           if self.mask is not None and self.pcl is not None:

                points = read_points_numpy(self.pcl, skip_nans=False, field_names=['x', 'y', 'z'])
                self.get_logger().info(f"Points shape {points.shape}")

                object_points = apply_mask_to_points(points, self.mask)

                closest_point = find_closest_point(origin=np.array([0.0, 0.0, 0.0]), points=object_points)
                self.get_logger().info(f"Closest point {closest_point}")
                self.send_transform(closest_point, 'closest_point', 'camera_link')

                self.publish_objects_pcl(object_points, 'camera_link')
                self.mask, self.pcl = None, None



def main(args=None):
    rclpy.init(args=args)

    vp_node = ClosestObjects()

    thread = threading.Thread(target=rclpy.spin, args=(vp_node,), daemon=True)
    thread.start()
    vp_node.run()

    vp_node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()