#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from practice_robotarm_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AnglesConverter(Node):

    def __init__(self):
        super().__init__('angle_conversion_service_server')
        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, 'euler_to_quaternion', self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, 'quaternion_to_euler', self.quaternionToEulerCallback)
        self.get_logger().info('Angle Conversion Services are ready.')
    
    def eulerToQuaternionCallback(self,req, res):
        self.get_logger().info('Received Euler angles: [%.2f, %.2f, %.2f]' % (req.roll, req.pitch, req.yaw))
        # Convert Euler angles to Quaternion
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        # Log the converted Quaternion
        self.get_logger().info('Converted Quaternion: [%.2f, %.2f, %.2f, %.2f]' % (res.x, res.y, res.z, res.w))
        return res

    def quaternionToEulerCallback(self, req, res):
        self.get_logger().info('Received Quaternion: [%.2f, %.2f, %.2f, %.2f]' % (req.x, req.y, req.z, req.w))
        # Convert Quaternion to Euler angles
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        # Log the converted Euler angles
        self.get_logger().info('Converted Euler angles: [%.2f, %.2f, %.2f]' % (res.roll, res.pitch, res.yaw))
        return res

def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()