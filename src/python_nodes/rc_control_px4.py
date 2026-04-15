#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, RCIn
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16MultiArray

PWM_MIN  = 1100
PWM_MID  = 1500
PWM_MAX  = 1900
RC_MIN   = 1000
RC_MAX   = 2000
RC_MID   = 1500
DEADZONE = 30

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

        self.joy_linear  = 0.0
        self.joy_angular = 0.0

        now = self.get_clock().now()

        self.last_rc_time  = now
        self.last_joy_time = now

        self.rc_timeout_sec  = 1.0
        self.joy_timeout_sec = 1.0

        self.create_subscription(State, '/mavros/state', self.cb_state, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self.cb_rc, 10)
        self.create_subscription(Joy, '/safe/joy', self.cb_joy, 10)

        self.pub_pwm = self.create_publisher(
            UInt16MultiArray, '/pwm_outputs', 10)

        self.create_timer(0.02, self.control_loop)

    def rc_to_float(self, raw):
        raw = max(RC_MIN, min(RC_MAX, int(raw)))
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
        self.last_rc_time = self.get_clock().now()

        if len(msg.channels) > 3:
            self.linear  = self.rc_to_float(msg.channels[1])
            self.angular = self.rc_to_float(msg.channels[3])

    def cb_joy(self, msg):
        self.last_joy_time = self.get_clock().now()

        if len(msg.axes) >= 2:
            self.joy_linear  = msg.axes[1]
            self.joy_angular = msg.axes[0]

    def apply_inversion(self, motor_idx, value_float):
        if MOTOR_INVERTED[motor_idx]:
            return self.float_to_us(-value_float)
        return self.float_to_us(value_float)

    def control_loop(self):
        pwm = [PWM_MID] * 4

        now = self.get_clock().now()

        dt_rc  = (now - self.last_rc_time).nanoseconds * 1e-9
        dt_joy = (now - self.last_joy_time).nanoseconds * 1e-9

        rc_alive  = dt_rc  < self.rc_timeout_sec
        joy_alive = dt_joy < self.joy_timeout_sec

        # 🔥 lógica correcta
        if self.armed and self.manual and rc_alive:
            linear  = self.linear
            angular = self.angular

        elif joy_alive:
            linear  = self.joy_linear
            angular = self.joy_angular

        else:
            linear  = 0.0
            angular = 0.0

        left  = linear + angular
        right = linear - angular

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