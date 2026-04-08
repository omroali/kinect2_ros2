#!/usr/bin/env python3
"""
Dynamic Camera TF Broadcaster

This node publishes TF transforms for Kinect cameras with positions that can be
adjusted dynamically at runtime using ROS 2 parameters.

Usage:
    ros2 run kinect2_bridge dynamic_camera_tf.py

    # Change camera 1 position at runtime:
    ros2 param set /dynamic_camera_tf camera1_x 1.0
    ros2 param set /dynamic_camera_tf camera1_y 0.5
    ros2 param set /dynamic_camera_tf camera1_yaw 0.785

    # Or use rqt_reconfigure for a GUI:
    ros2 run rqt_reconfigure rqt_reconfigure
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class DynamicCameraTF(Node):
    def __init__(self):
        super().__init__('dynamic_camera_tf')

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameter descriptors for nice GUI in rqt_reconfigure
        position_range = FloatingPointRange(
            from_value=-10.0,
            to_value=10.0,
            step=0.0  # 0 means no step constraint
        )
        angle_range = FloatingPointRange(
            from_value=-3.15,
            to_value=3.15,
            step=0.0  # 0 means no step constraint
        )

        pos_desc = ParameterDescriptor(
            description='Position in meters',
            floating_point_range=[position_range]
        )
        angle_desc = ParameterDescriptor(
            description='Angle in radians',
            floating_point_range=[angle_range]
        )

        # Declare parameters for Camera 1
        self.declare_parameter('camera1_frame', 'kinect2_1_link')
        self.declare_parameter('camera1_x', 0.0, pos_desc)
        self.declare_parameter('camera1_y', 0.0, pos_desc)
        self.declare_parameter('camera1_z', 0.0, pos_desc)
        self.declare_parameter('camera1_roll', 0.0, angle_desc)
        self.declare_parameter('camera1_pitch', 0.0, angle_desc)
        self.declare_parameter('camera1_yaw', 0.0, angle_desc)
        self.declare_parameter('camera1_enabled', True)

        # Declare parameters for Camera 2
        self.declare_parameter('camera2_frame', 'kinect2_2_link')
        self.declare_parameter('camera2_x', 1.0, pos_desc)
        self.declare_parameter('camera2_y', 0.0, pos_desc)
        self.declare_parameter('camera2_z', 0.0, pos_desc)
        self.declare_parameter('camera2_roll', 0.0, angle_desc)
        self.declare_parameter('camera2_pitch', 0.0, angle_desc)
        self.declare_parameter('camera2_yaw', -0.5236, angle_desc)  # -30 degrees
        self.declare_parameter('camera2_enabled', True)

        # World frame
        self.declare_parameter('world_frame', 'map')

        # Publish rate
        self.declare_parameter('publish_rate', 50.0)

        # Register parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Load initial parameters
        self.load_parameters()

        # Create timer for publishing transforms
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.broadcast_transforms)

        self.get_logger().info('Dynamic Camera TF Broadcaster started')
        self.get_logger().info('Use "ros2 param set" or rqt_reconfigure to adjust camera positions')
        self.log_camera_positions()

    def load_parameters(self):
        """Load all parameters into instance variables."""
        self.world_frame = self.get_parameter('world_frame').value

        self.camera1 = {
            'frame': self.get_parameter('camera1_frame').value,
            'x': self.get_parameter('camera1_x').value,
            'y': self.get_parameter('camera1_y').value,
            'z': self.get_parameter('camera1_z').value,
            'roll': self.get_parameter('camera1_roll').value,
            'pitch': self.get_parameter('camera1_pitch').value,
            'yaw': self.get_parameter('camera1_yaw').value,
            'enabled': self.get_parameter('camera1_enabled').value,
        }

        self.camera2 = {
            'frame': self.get_parameter('camera2_frame').value,
            'x': self.get_parameter('camera2_x').value,
            'y': self.get_parameter('camera2_y').value,
            'z': self.get_parameter('camera2_z').value,
            'roll': self.get_parameter('camera2_roll').value,
            'pitch': self.get_parameter('camera2_pitch').value,
            'yaw': self.get_parameter('camera2_yaw').value,
            'enabled': self.get_parameter('camera2_enabled').value,
        }

    def parameter_callback(self, params):
        """Handle parameter changes."""
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')

            # Update the appropriate camera dictionary
            if param.name.startswith('camera1_'):
                key = param.name.replace('camera1_', '')
                if key in self.camera1:
                    self.camera1[key] = param.value
            elif param.name.startswith('camera2_'):
                key = param.name.replace('camera2_', '')
                if key in self.camera2:
                    self.camera2[key] = param.value
            elif param.name == 'world_frame':
                self.world_frame = param.value

        self.log_camera_positions()
        return SetParametersResult(successful=True)

    def log_camera_positions(self):
        """Log current camera positions."""
        if self.camera1['enabled']:
            self.get_logger().info(
                f"Camera 1 ({self.camera1['frame']}): "
                f"pos=({self.camera1['x']:.2f}, {self.camera1['y']:.2f}, {self.camera1['z']:.2f}), "
                f"rpy=({self.camera1['roll']:.2f}, {self.camera1['pitch']:.2f}, {self.camera1['yaw']:.2f})"
            )
        if self.camera2['enabled']:
            self.get_logger().info(
                f"Camera 2 ({self.camera2['frame']}): "
                f"pos=({self.camera2['x']:.2f}, {self.camera2['y']:.2f}, {self.camera2['z']:.2f}), "
                f"rpy=({self.camera2['roll']:.2f}, {self.camera2['pitch']:.2f}, {self.camera2['yaw']:.2f})"
            )

    def create_transform(self, camera_config: dict) -> TransformStamped:
        """Create a TransformStamped message from camera configuration."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = camera_config['frame']

        t.transform.translation.x = float(camera_config['x'])
        t.transform.translation.y = float(camera_config['y'])
        t.transform.translation.z = float(camera_config['z'])

        qx, qy, qz, qw = quaternion_from_euler(
            camera_config['roll'],
            camera_config['pitch'],
            camera_config['yaw']
        )
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        return t

    def broadcast_transforms(self):
        """Broadcast transforms for all enabled cameras."""
        transforms = []

        if self.camera1['enabled']:
            transforms.append(self.create_transform(self.camera1))

        if self.camera2['enabled']:
            transforms.append(self.create_transform(self.camera2))

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCameraTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
