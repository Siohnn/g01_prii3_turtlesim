#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class DrawNumberNode(Node):
    def __init__(self):
        super().__init__('draw_number_01')
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen = self.create_client(SetPen, '/turtle1/set_pen')

        # Esperar a que los servicios estén disponibles
        while not self.teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /turtle1/teleport_absolute...')
        while not self.set_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /turtle1/set_pen...')

        self.get_logger().info("Dibujando el número 01...")
        time.sleep(1.0)
        self.draw_number_01()
        self.get_logger().info("Dibujo completado ✅")

    def call_teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self.teleport.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.2)

    def call_set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.set_pen.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)

    def draw_number_01(self):
        # Dibujar "0"
        self.call_set_pen(255, 0, 0, 3, 0)
        self.call_teleport(4.0, 5.0, 0.0)
        self.call_teleport(4.0, 7.0, 0.0)
        self.call_teleport(6.0, 7.0, 0.0)
        self.call_teleport(6.0, 5.0, 0.0)
        self.call_teleport(4.0, 5.0, 0.0)

        # Mover sin dibujar hacia el "1"
        self.call_set_pen(0, 0, 0, 3, 1)
        self.call_teleport(7.5, 5.0, 0.0)

        # Dibujar "1"
        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(7.5, 7.0, 0.0)
        self.call_set_pen(0, 0, 0, 3, 1)

def main(args=None):
    rclpy.init(args=args)
    node = DrawNumberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

