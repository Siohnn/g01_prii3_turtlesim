#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

def quaternion_from_yaw(yaw):
    """Convierte yaw (rad) a quaternion (x,y,z,w)"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw/2.0)
    q.w = math.cos(yaw/2.0)
    return q

class NavigationClient(Node):
    def __init__(self):
        super().__init__("navigation_client")
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Puntos destino (x, y, yaw)
        self.destinos = {
            1: (-1.67, 6.43, 0.0),
            2: (-0.81, 15.45, 0.0),
            3: (5.57, 15.47, 0.0)
        }

        # Establecer posición inicial
        self.set_initial_pose(0.0, 0.0, 0.0)

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = quaternion_from_yaw(yaw)

        # Covarianza pequeña, asumiendo certeza
        msg.pose.covariance = [0.0]*36
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25  # y
        msg.pose.covariance[35] = 0.06853892326654787  # yaw

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"Posición inicial establecida en ({x}, {y}, yaw={yaw})")

    def enviar_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = quaternion_from_yaw(yaw)

        self.get_logger().info(f"Enviando robot a ({x}, {y})")

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return

        # Esperar a que la navegación termine
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        self.get_logger().info(f"Navegación terminada: {result.status}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()

    try:
        while rclpy.ok():
            print("""
Elige destino:
1 → (-1.67, 6.43)
2 → (-0.81, 15.45)
3 → (5.57, 15.47)
0 → Salir
""")
            try:
                n = int(input("Introduce un número: "))
            except ValueError:
                print("Entrada no válida, introduce un número.")
                continue

            if n == 0:
                print("Saliendo...")
                break
            elif n in node.destinos:
                x, y, yaw = node.destinos[n]
                node.enviar_goal(x, y, yaw)
            else:
                print("Número no válido")

    except KeyboardInterrupt:
        print("\nCerrando nodo por Ctrl+C...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

