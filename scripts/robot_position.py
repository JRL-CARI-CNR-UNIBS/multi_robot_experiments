#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
# from tf_transformations import euler_from_quaternion

class TfListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener_node')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)  # Call on_timer every second

    def on_timer(self):

        try:
            now = rclpy.time.Time()
            all_frames = self.buffer.all_frames_as_string()
            self.get_logger().info(f"All frames:\n{all_frames}")
            trans = self.buffer.lookup_transform('map', 'base_link', now)

            # Print the translation (x, y, z)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            self.get_logger().info(f"Translation: x: {x}, y: {y}, z: {z}")

            # Print the quaternion (x, y, z, w)
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            self.get_logger().info(f"Quaternion: qx: {qx}, qy: {qy}, qz: {qz}, qw: {qw}")

            # Convert quaternion to Euler angles (yaw, pitch, roll)
            # (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            # self.get_logger().info(f"Euler angles: yaw: {yaw}, pitch: {pitch}, roll: {roll}")

        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
