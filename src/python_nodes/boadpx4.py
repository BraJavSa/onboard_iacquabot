#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, RCIn
from std_msgs.msg import UInt16MultiArray

PWM_MIN  = 1100
PWM_MID  = 1500
PWM_MAX  = 1900
RC_MIN   = 1000
RC_MAX   = 2000
RC_MID   = 1500
DEADZONE = 30

class PWMController(Node):
    def __init__(self):
        super().__init__('pwm_controller')

        self.armed   = False
        self.manual  = False
        self.linear  = 0.0
        self.angular = 0.0

        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.cb_state, 10)
        self.sub_rc = self.create_subscription(
            RCIn, '/mavros/rc/in', self.cb_rc, 10)

        self.pub_pwm = self.create_publisher(
            UInt16MultiArray, '/pwm_outputs', 10)

        # Publicar a 50Hz
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info('PWM Controller iniciado')

    def rc_to_float(self, raw):
        raw = max(RC_MIN, min(RC_MAX, raw))
        centered = raw - RC_MID
        if abs(centered) < DEADZONE:
            return 0.0
        if centered > 0:
            return (centered - DEADZONE) / (RC_MAX - RC_MID - DEADZONE)
        else:
            return (centered + DEADZONE) / (RC_MID - RC_MIN - DEADZONE)

    def float_to_us(self, val):
        val = max(-1.0, min(1.0, val))
        return int(PWM_MID + val * (PWM_MAX - PWM_MID))

    def cb_state(self, msg):
        self.armed  = msg.armed
        self.manual = (msg.mode == 'MANUAL')

        if not self.armed or not self.manual:
            self.linear  = 0.0
            self.angular = 0.0

    def cb_rc(self, msg):
        if len(msg.channels) > 3:
            # Canal 2 (índice 1) = Pitch → linear
            # Canal 4 (índice 3) = Yaw   → angular
            self.linear  = self.rc_to_float(msg.channels[1])
            self.angular = self.rc_to_float(msg.channels[3])

    def control_loop(self):
        if self.armed and self.manual:
            left  = self.linear + self.angular
            right = self.linear - self.angular

            # Normalizar
            maxv = max(abs(left), abs(right))
            if maxv > 1.0:
                left  /= maxv
                right /= maxv

            left_us  = self.float_to_us(left)
            right_us = self.float_to_us(right)
        else:
            left_us  = PWM_MID
            right_us = PWM_MID

        msg = UInt16MultiArray()
        # [left_a, left_b, right_a, right_b]
        msg.data = [left_us, left_us, right_us, right_us]
        self.pub_pwm.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PWMController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()