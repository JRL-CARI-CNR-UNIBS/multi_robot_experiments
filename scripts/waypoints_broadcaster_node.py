#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from typing import List

class WaypointsBroadcasterNode(Node):
    def __init__(self):
        super().__init__('waypoints_broadcaster_node')
        
        self.declare_parameter('waypoints', rclpy.Parameter.Type.STRING_ARRAY)
        waypoints_name = self.get_parameter('waypoints').get_parameter_value().string_array_value

        self.waypoints = self._get_waypoints(waypoints_name)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_waypoints_transforms()

    def publish_waypoints_transforms(self):
        
        transforms = []
        for waypoint_name, waypoint_3d in self.waypoints.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = waypoint_name

            self.get_logger().info(f"Waypoint: {waypoint_name}")
            self.get_logger().info(f"Waypoint: {waypoint_3d['position']}")

            transform.transform.translation.x = waypoint_3d['position']['x']
            transform.transform.translation.y = waypoint_3d['position']['y']
            transform.transform.translation.z = waypoint_3d['position']['z']
            transform.transform.rotation.x = waypoint_3d['orientation']['x']
            transform.transform.rotation.y = waypoint_3d['orientation']['x']
            transform.transform.rotation.z = waypoint_3d['orientation']['y']
            transform.transform.rotation.w = waypoint_3d['orientation']['z']
            
            transforms.append(transform)
        
        self.tf_broadcaster.sendTransform(transforms)

    def _get_waypoints(self, waypoints_name: List[str]):
        waypoints = {}
        for waypoint in waypoints_name:
            self.get_logger().info(f"Waypoint: {waypoint}")
            self.declare_parameter(f'{waypoint}.position.x', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{waypoint}.position.y', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{waypoint}.position.z', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{waypoint}.orientation.x', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{waypoint}.orientation.y', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{waypoint}.orientation.z', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{waypoint}.orientation.w', rclpy.Parameter.Type.DOUBLE)

            position_x = self.get_parameter_or(f'{waypoint}.position.x', None).value
            position_y = self.get_parameter_or(f'{waypoint}.position.y', None).value
            position_z = self.get_parameter_or(f'{waypoint}.position.z', None).value
            orientation_x = self.get_parameter_or(f'{waypoint}.orientation.x', None).value
            orientation_y = self.get_parameter_or(f'{waypoint}.orientation.y', None).value
            orientation_z = self.get_parameter_or(f'{waypoint}.orientation.z', None).value
            orientation_w = self.get_parameter_or(f'{waypoint}.orientation.w', None).value

            if None not in (position_x, 
                            position_y, 
                            position_z, 
                            orientation_x, 
                            orientation_y, 
                            orientation_z, 
                            orientation_w):
                waypoints[waypoint] = {
                    'position': {
                        'x': position_x,
                        'y': position_y,
                        'z': position_z,
                    },
                    'orientation': {
                        'x': orientation_x,
                        'y': orientation_y,
                        'z': orientation_z,
                        'w': orientation_w,
                    }
                }
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
