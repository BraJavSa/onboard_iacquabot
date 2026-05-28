#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyToSafeJoy(Node):
    def __init__(self):
        super().__init__('joy_to_safe_joy')
        
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Joy, '/safe/joy', 10)
        
        self.last_msg = None
        
        self.create_timer(1.0 / 30.0, self.timer_callback)
        
        self.get_logger().info('Nodo joy_to_safe_joy iniciado. Publicando /safe/joy a 30 Hz')

    def joy_callback(self, msg):
        self.last_msg = msg

    def timer_callback(self):
        if self.last_msg is None:
            return
        
        msg = self.last_msg
        safe_msg = Joy()
        safe_msg.header = msg.header
        safe_msg.buttons = msg.buttons
        
        axes_list = list(msg.axes)
        
        if len(axes_list) >= 4:
            axes_list[0] = msg.axes[3]
        
        safe_msg.axes = axes_list
        self.pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToSafeJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()