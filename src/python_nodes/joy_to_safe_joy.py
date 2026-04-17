#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyToSafeJoy(Node):
    def __init__(self):
        super().__init__('joy_to_safe_joy')
        
        # Suscribirse al tópico /joy
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publicar en el tópico /safe/joy
        self.pub = self.create_publisher(Joy, '/safe/joy', 10)
        
        self.get_logger().info('Nodo joy_to_safe_joy iniciado. Reenviando de /joy a /safe/joy')

    def joy_callback(self, msg):
        # Crear un nuevo mensaje modificado
        safe_msg = Joy()
        safe_msg.header = msg.header
        safe_msg.buttons = msg.buttons
        
        # Convertimos la tupla de ejes a lista para poder modificarla
        axes_list = list(msg.axes)
        
        # rc_control_px4.py espera:
        # Avance (linear)  -> axes[1]
        # Rotación (angular) -> axes[0]
        
        # Por defecto los mandos usan:
        # Stick Izquierdo Arriba/Abajo -> axes[1] (Avance)
        # Stick Derecho Izq/Der        -> axes[3] (o a veces axes[2])
        
        if len(axes_list) >= 4:
            # Reemplazamos el índice 0 (que lee tu script) por el valor del stick derecho (índice 3)
            # Si tu mando usa otro índice (ej. PS usa diferente a veces), podrías cambiar el 3 por 2.
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
