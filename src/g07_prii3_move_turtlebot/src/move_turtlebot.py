#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class DrawNumber(Node):
    def __init__(self):
        super().__init__('draw_number')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Nodo draw_number iniciado. Dibujando número 07...')
        time.sleep(1)
        self.draw_number()

    def move_straight(self, speed=0.15, duration=1.0):
        msg = Twist()
        msg.linear.x = speed
        self.publish_for_duration(msg, duration)

    def turn(self, angular_speed=0.5, angle=math.pi / 2):
        """Gira un ángulo dado (en radianes) con una velocidad angular."""
        duration = abs(angle / angular_speed)
        msg = Twist()
        msg.angular.z = angular_speed if angle > 0 else -angular_speed
        self.publish_for_duration(msg, duration)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)
        time.sleep(0.5)

    def publish_for_duration(self, msg, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def draw_number(self):
        # Escala: 2 unidades de alto, 1 de ancho
        # Aprox. 1 unidad ≈ 2 segundos a 0.15 m/s  →  2 unidades ≈ 4 segundos

        # === Dibuja el "0" ===
        self.get_logger().info('Dibujando 0...')
        for _ in range(2):  # Dos lados largos
            self.move_straight(0.15, 4.0)  # Alto (2 unidades)
            self.turn(0.5, math.pi / 2)    # Giro 90°
            self.move_straight(0.15, 2.0)  # Ancho (1 unidad)
            self.turn(0.5, math.pi / 2)    # Giro 90°

        self.get_logger().info('Traza del 0 acabada.')

        # === Separación entre dígitos ===
        self.get_logger().info('Distancia de separación...')
        self.turn(0.5, -math.pi / 2)   # gira a la derecha
        self.move_straight(0.15, 3.0)  # separa unos 1.5 unidades aprox.
        self.turn(0.5, math.pi / 2)    # vuelve al frente
        self.get_logger().info('Distancia de separación concluida.')

        # === Dibuja el "7" ===
        self.get_logger().info('Dibujando 7...')
        self.move_straight(0.25, 4.0)
        self.turn(0.5, math.pi / 2)    # gira izquierda
        self.move_straight(0.15, 2.0)   # parte superior (1 unidad)
        self.get_logger().info('Trazo del número 07 completado.')
        self.stop()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DrawNumber()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

