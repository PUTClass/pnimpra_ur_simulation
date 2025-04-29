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
    closest_point = None
    ### Find the closest point from the origin point.
    ### The closest point is determined given the Euclidean distance from all points to the origin point.
    ### You can use np.linalg.norm function to calculate the Frobenius norm (square of Euclidean distance)
    ### Finding the minimum in a data structure can be also realized with numpy.
    ### All of the above functions are optional and you can use simple loops to find the closest point.
    return closest_point

def apply_mask_to_points(points, mask):
    points = None
    ### point and mask are data structures with the same amount of data points
    ### Corresponding pixels from the mask can be used to create a new list of points, where
    ### only objects are visible
    ### np.nonzero(x) returns indices of all nonzero values in x. It might be used to avoid loop functions.
    return points


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
        
        ### Hard-coded quaternion rotation to assign a reachable orientation to 'closest_point' TF
        q_x = quaternion_from_euler(0.0,  0.0, np.pi/2)
        q_y = quaternion_from_euler(0.0, np.pi/2, 0.0)

        q = quaternion_multiply(q_x, q_y)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_objects_pcl(self, points, frame_id):
        pc2 = None
        ### Create a point cloud with objects.
        ### You can use create_cloud_xyz32(header: Header, points: Iterable) function to create a PointCloud2
        ### structure. The header must be filled with a correct name of a transform from TF tree and a timestamp.
        ### self.get_clock().now().to_msg() denoted a present moment in time.
        self.point_cloud_publisher.publish(pc2)
    

    def run(self):
        while rclpy.ok():
           if self.mask is not None and self.pcl is not None:

                points = read_points_numpy(self.pcl, skip_nans=False, field_names=['x', 'y', 'z'])

                # object_points = apply_mask_to_points(points, self.mask)
                # closest_point = find_closest_point(o)
                # self.send_transform(closest_point, 'closest_point', 'some_transform')
                # self.publish_objects_pcl(object_points, 'camera_link')
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