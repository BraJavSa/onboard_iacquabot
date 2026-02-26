#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, ManualControl
from std_msgs.msg import UInt16MultiArray

PWM_MIN  = 1100
PWM_MID  = 1500
PWM_MAX  = 1900
DEADZONE = 0.03  # normalizado, equivale a ~30 en escala 1000

MOTOR_FL = 1
MOTOR_FR = 3
MOTOR_BL = 0
MOTOR_BR = 2

MOTOR_INVERTED = [True, True, False, False]


class PWMController(Node):
    def __init__(self):
        super().__init__('pwm_controller')

        self.armed   = False
        self.manual  = False
        self.linear  = 0.0
        self.angular = 0.0

        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.cb_state, 10)
        self.sub_manual = self.create_subscription(
            ManualControl, '/mavros/manual_control/control', self.cb_manual, 10)

        self.pub_pwm = self.create_publisher(
            UInt16MultiArray, '/pwm_outputs', 10)

        self.create_timer(0.02, self.control_loop)
        self.get_logger().info('PWM Controller (ManualControl) iniciado')
        self.get_logger().info(f'FL={MOTOR_FL} FR={MOTOR_FR} BL={MOTOR_BL} BR={MOTOR_BR}')
        self.get_logger().info(f'Invertidos: {MOTOR_INVERTED}')

    def apply_deadzone(self, val):
        if abs(val) < DEADZONE:
            return 0.0
        return max(-1.0, min(1.0, val))

    def float_to_us(self, val):
        val = max(-1.0, min(1.0, val))
        return int(PWM_MID + val * (PWM_MAX - PWM_MID))

    def cb_state(self, msg):
        self.armed  = msg.armed
        self.manual = (msg.mode == 'MANUAL')
        if not self.armed or not self.manual:
            self.linear  = 0.0
            self.angular = 0.0

    def cb_manual(self, msg):
        if not self.armed or not self.manual:
            return
        self.linear  = self.apply_deadzone(msg.z)   # throttle bidireccional
        self.angular = self.apply_deadzone(msg.y)   # giro

    def apply_inversion(self, motor_idx, value_float):
        if MOTOR_INVERTED[motor_idx]:
            return self.float_to_us(-value_float)
        return self.float_to_us(value_float)

    def control_loop(self):
        pwm = [PWM_MID] * 4

        if self.armed and self.manual:
            left  = self.linear + self.angular
            right = self.linear - self.angular

            maxv = max(abs(left), abs(right), 1e-9)
            if maxv > 1.0:
                left  /= maxv
                right /= maxv

            pwm[MOTOR_FL] = self.apply_inversion(MOTOR_FL, left)
            pwm[MOTOR_FR] = self.apply_inversion(MOTOR_FR, right)
            pwm[MOTOR_BL] = self.apply_inversion(MOTOR_BL, left)
            pwm[MOTOR_BR] = self.apply_inversion(MOTOR_BR, right)

        msg = UInt16MultiArray()
        msg.data = pwm
        self.pub_pwm.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PWMController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()