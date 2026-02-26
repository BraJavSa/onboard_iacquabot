#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import time
import math
import subprocess
import re
import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float64, Int32
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped

KNOT_TO_MPS = 0.514444
NM_TO_M = 1852.0

class NmeaTcpBridge(Node):
    def __init__(self):
        super().__init__('nmea_tcp_bridge')

        # Parámetros configurables
        self.declare_parameter('target_mac', '60:e8:5b:a1:c7:5b')
        self.declare_parameter('port', 10110)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('reconnect_seconds', 5.0)

        self.target_mac = self.get_parameter('target_mac').value.lower()
        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_seconds = float(self.get_parameter('reconnect_seconds').value)

        # Publishers (prefijo /lowrance/)
        self.pub_raw = self.create_publisher(String, '/lowrance/nmea_raw', 100)
        self.pub_fix = self.create_publisher(NavSatFix, '/lowrance/gps/fix', 20)
        self.pub_vel = self.create_publisher(TwistStamped, '/lowrance/gps/vel', 20)

        self.pub_hdop = self.create_publisher(Float32, '/lowrance/gps/hdop', 10)
        self.pub_vdop = self.create_publisher(Float32, '/lowrance/gps/vdop', 10)
        self.pub_pdop = self.create_publisher(Float32, '/lowrance/gps/pdop', 10)
        self.pub_sats_used = self.create_publisher(Int32, '/lowrance/gps/sats_used', 10)
        self.pub_sats_view = self.create_publisher(Int32, '/lowrance/gps/sats_in_view', 10)
        self.pub_time_ref = self.create_publisher(TimeReference, '/lowrance/gps/time_ref', 10)

        self.pub_hdg_mag = self.create_publisher(Float64, '/lowrance/heading/mag_deg', 20)
        self.pub_hdg_true = self.create_publisher(Float64, '/lowrance/heading/true_deg', 20)

        self.pub_depth = self.create_publisher(Float32, '/lowrance/depth', 20)
        self.pub_temp = self.create_publisher(Float32, '/lowrance/water_temp', 10)
        self.pub_water_speed = self.create_publisher(Float32, '/lowrance/water_speed_mps', 10)
        self.pub_odo_total = self.create_publisher(Float64, '/lowrance/odo/water_total_m', 5)
        self.pub_odo_trip = self.create_publisher(Float64, '/lowrance/odo/water_trip_m', 5)

        # Thread de gestión de conexión
        self._stop = threading.Event()
        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.get_logger().info(f'Nodo iniciado. Buscando MAC: {self.target_mac}')

    def get_ip_from_mac(self, mac):
        """Escanea la tabla ARP del sistema para encontrar la IP asociada a la MAC."""
        try:
            output = subprocess.check_output(['arp', '-n'], stderr=subprocess.DEVNULL).decode()
            # Expresión regular para extraer IP basada en la MAC
            pattern = r'(\d+\.\d+\.\d+\.\d+).*' + re.escape(mac)
            match = re.search(pattern, output, re.IGNORECASE)
            if match:
                return match.group(1)
        except Exception as e:
            self.get_logger().error(f'Error al consultar tabla ARP: {e}')
        return None

    def _reader_loop(self):
        """Bucle principal de conexión y lectura."""
        while not self._stop.is_set():
            current_ip = self.get_ip_from_mac(self.target_mac)
            
            if not current_ip:
                self.get_logger().warn(f'Dispositivo {self.target_mac} no detectado en ARP. Reintentando...')
                time.sleep(self.reconnect_seconds)
                continue

            sock = None
            try:
                self.get_logger().info(f'Conectando a {current_ip}:{self.port} (MAC: {self.target_mac})')
                sock = socket.create_connection((current_ip, self.port), timeout=10.0)
                f = sock.makefile('r', encoding='ascii', errors='ignore')
                self.get_logger().info(f'Conexión establecida con Lowrance en {current_ip}')
                
                for line in f:
                    if self._stop.is_set(): break
                    line = line.strip()
                    if not line: continue
                    self._handle_sentence(line)
            except Exception as e:
                self.get_logger().warn(f'Error de conexión: {e}. Reintentando en {self.reconnect_seconds}s')
                time.sleep(self.reconnect_seconds)
            finally:
                if sock:
                    sock.close()

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

    def _handle_sentence(self, sentence: str):
        """Parsea las sentencias NMEA y publica en los tópicos correspondientes."""
        self.pub_raw.publish(String(data=sentence))
        try:
            msg = pynmea2.parse(sentence, check=True)
            stype = msg.sentence_type.upper()
        except:
            return

        header = self.get_clock().now().to_msg()

        # GPS Fixes (GLL, RMC)
        if stype in ['GLL', 'RMC']:
            fix = NavSatFix()
            fix.header.stamp = header
            fix.header.frame_id = self.frame_id
            fix.status.service = NavSatStatus.SERVICE_GPS
            try:
                fix.latitude = float(msg.latitude)
                fix.longitude = float(msg.longitude)
                self.pub_fix.publish(fix)
            except: pass

        # Velocidad y Rumbo sobre el fondo (VTG)
        if stype == 'VTG':
            tw = TwistStamped()
            tw.header.stamp = header
            tw.header.frame_id = self.frame_id
            try:
                tw.twist.linear.x = float(msg.spd_over_grnd_kts) * KNOT_TO_MPS
                tw.twist.angular.z = math.radians(float(msg.true_track))
                self.pub_vel.publish(tw)
            except: pass

        # Rumbo (HDG, THS)
        if stype == 'HDG':
            try:
                self.pub_hdg_mag.publish(Float64(data=float(msg.heading)))
            except: pass
        
        if stype == 'THS':
            try:
                self.pub_hdg_true.publish(Float64(data=float(msg.heading)))
            except: pass

        # Profundidad (DPT, DBT)
        if stype == 'DPT':
            try:
                self.pub_depth.publish(Float32(data=float(msg.depth)))
            except: pass
        elif stype == 'DBT':
            try:
                self.pub_depth.publish(Float32(data=float(msg.meters)))
            except: pass

        # Temperatura del agua (MTW)
        if stype == 'MTW':
            try:
                self.pub_temp.publish(Float32(data=float(msg.temperature)))
            except: pass

        # Velocidad en el agua (VHW)
        if stype == 'VHW':
            try:
                self.pub_water_speed.publish(Float32(data=float(msg.spd_knots) * KNOT_TO_MPS))
            except: pass

        # Odometría (VLW)
        if stype == 'VLW':
            try:
                self.pub_odo_total.publish(Float64(data=float(msg.total_cum_dist) * NM_TO_M))
                self.pub_odo_trip.publish(Float64(data=float(msg.trip_dist) * NM_TO_M))
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = NmeaTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()