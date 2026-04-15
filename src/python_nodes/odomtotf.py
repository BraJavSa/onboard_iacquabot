#!/usr/bin/env python3
"""
mavros_tf_publisher.py
Publica el TF odom -> base_link leyendo /mavros/odometry/in.
Necesario cuando PX4 v1.12.3 no publica LOCAL_POSITION_NED
pero si publica ODOMETRY.

Uso:
    python3 mavros_tf_publisher.py

O desde un launch file:
    Node(package='onboard_iacquabot',
         executable='mavros_tf_publisher',
         name='mavros_tf_publisher')
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MavrosTFPublisher(Node):

    def __init__(self):
        super().__init__('mavros_tf_publisher')

        # Parametros configurables
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link_odom')
        self.declare_parameter('odom_topic', '/mavros/odometry/in')

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame  = self.get_parameter('child_frame').value
        odom_topic        = self.get_parameter('odom_topic').value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'mavros_tf_publisher iniciado: '
            f'{self.parent_frame} -> {self.child_frame} '
            f'desde {odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        # Usar el mismo timestamp que el mensaje de odometria
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame

        # Posicion
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Orientacion
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()