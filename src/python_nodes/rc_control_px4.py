#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16MultiArray

PWM_MID = 1500
PWM_MAX = 1900

DEADZONE = 0.05  # 5% deadzone en ejes del joystick

# Canal 0 = derecha, Canal 1 = izquierda (según rc/out)
CH_RIGHT = 0
CH_LEFT  = 1

JOY_TIMEOUT_SEC   = 1.0
RCOUT_TIMEOUT_SEC = 1.0


class PWMController(Node):
    def __init__(self):
        super().__init__('pwm_controller')

        self.joy_linear  = 0.0
        self.joy_angular = 0.0

        self.ap_right = PWM_MID
        self.ap_left  = PWM_MID

        self.last_joy_time   = None
        self.last_rcout_time = None

        self.create_subscription(RCOut, '/mavros/rc/out', self.cb_rc_out, 10)
        self.create_subscription(Joy,   '/safe/joy',      self.cb_joy,    10)

        self.pub_pwm = self.create_publisher(
            UInt16MultiArray, '/pwm_outputs', 10)

        self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.get_logger().info('PWM Controller iniciado')

    def apply_deadzone(self, val):
        if abs(val) < DEADZONE:
            return 0.0
        return val

    def float_to_us(self, val):
        val = max(-1.0, min(1.0, val))
        return int(PWM_MID + val * (PWM_MAX - PWM_MID))

    def cb_rc_out(self, msg):
        self.last_rcout_time = self.get_clock().now()
        if len(msg.channels) > CH_LEFT:
            self.ap_right = msg.channels[CH_RIGHT]
            self.ap_left  = msg.channels[CH_LEFT]

    def cb_joy(self, msg):
        self.last_joy_time = self.get_clock().now()
        if len(msg.axes) >= 2:
            self.joy_linear  = self.apply_deadzone(msg.axes[1])
            self.joy_angular = self.apply_deadzone(-msg.axes[2])

    def is_alive(self, last_time, timeout):
        if last_time is None:
            return False
        dt = (self.get_clock().now() - last_time).nanoseconds * 1e-9
        return dt < timeout

    def control_loop(self):
        joy_alive   = self.is_alive(self.last_joy_time,   JOY_TIMEOUT_SEC)
        rcout_alive = self.is_alive(self.last_rcout_time, RCOUT_TIMEOUT_SEC)

        if joy_alive:
            # Prioridad máxima: joystick
            linear  = self.joy_linear
            angular = self.joy_angular

            left  = linear + angular
            right = linear - angular

            maxv = max(abs(left), abs(right), 1e-9)
            if maxv > 1.0:
                left  /= maxv
                right /= maxv

            pwm_right = self.float_to_us(right)
            pwm_left  = self.float_to_us(left)

        elif rcout_alive:
            # Fallback: salidas del autopiloto
            pwm_right = self.ap_right
            pwm_left  = self.ap_left

        else:
            # Sin señal de ninguno → neutro
            pwm_right = PWM_MID
            pwm_left  = PWM_MID

        msg = UInt16MultiArray()
        msg.data = [pwm_right, pwm_left]
        self.pub_pwm.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PWMController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()