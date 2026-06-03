#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
import struct
import threading

class MavlinkBridge(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')

        # Conexión hacia QGC
        self.mav = mavutil.mavlink_connection(
            'udpout:127.0.0.1:14550',
            source_system=1,
            source_component=1
        )
        self.get_logger().info('Conexión MAVLink hacia QGC lista')

        # Enviar heartbeats periódicos
        self.timer = self.create_timer(1.0, self.send_heartbeat)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.count = 0
        self.sub = self.create_subscription(
            Mavlink, '/uas1/mavlink_source', self.callback, qos)

    def send_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )

    def callback(self, msg):
        self.count += 1
        if self.count <= 5 or self.count % 1000 == 0:
            self.get_logger().info(f'#{self.count} msgid={msg.msgid}')

        # Reconstruir bytes crudos y enviar directo
        payload = b''
        for p in msg.payload64:
            payload += struct.pack('<Q', p)
        payload = payload[:msg.len]

        self.mav.write(bytes([
            msg.magic, msg.len,
            msg.incompat_flags, msg.compat_flags,
            msg.seq, msg.sysid, msg.compid,
            msg.msgid & 0xFF,
            (msg.msgid >> 8) & 0xFF,
            (msg.msgid >> 16) & 0xFF
        ]) + payload + struct.pack('<H', msg.checksum))

def main():
    rclpy.init()
    node = MavlinkBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
