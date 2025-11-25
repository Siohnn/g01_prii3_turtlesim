#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class AvoidColisions(Node):
    def __init__(self):
        super().__init__('avoid_colisions')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # El callback del láser se procesa de forma ASÍNCRONA.
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.min_distance = 0.3  # Distancia mínima de seguridad (m)
        self.obstacle_detected = False

        # --- Variables de Control de Estado ---
        self.current_twist = Twist() # Almacena el comando de velocidad actual
        self.state = 'IDLE'          # Estados: 'IDLE', 'MOVING', 'FINISHED'
        self.end_time = 0.0          # Tiempo final del comando de movimiento actual

        # --- Secuencia de Dibujo ---
        self.drawing_sequence = iter(self._get_drawing_commands()) # Iterador de comandos
        self.drawing_active = True

        # Crea un timer que se ejecuta cada 50ms (20Hz)
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(' Nodo avoid_colisions iniciado. Control por Timer activo.')
        self.get_logger().info('Robot dibujará el número 07 evitando colisiones.')
        time.sleep(1)


    def laser_callback(self, msg: LaserScan):
        """Detecta cualquier obstáculo dentro del rango mínimo y actualiza el estado."""
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return

        min_range = min(valid_ranges)
        
        if min_range < self.min_distance:
            if not self.obstacle_detected:
                # La detención real la ejecuta el _control_loop, pero logueamos aquí
                self.get_logger().warn(f' Obstáculo detectado a {min_range:.2f} m. Deteniendo robot.')
            self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.get_logger().info(' Obstáculo fuera de rango, retomando movimiento.')
            self.obstacle_detected = False

    
    def _get_drawing_commands(self):
        """Define la secuencia de comandos (lineal_vel, angular_vel, duración_s)."""
        commands = []
        self.get_logger().info('Preparando comandos para dibujar 0...')
        
        # Dibujando 0 (dos rectángulos de 4x2)
        angular_duration = math.pi / 2 / 0.5 # Duración para un giro de 90 grados a 0.5 rad/s
        for _ in range(2):
            commands.extend([
                (0.15, 0.0, 4.0),           # Adelante
                (0.0, 0.5, angular_duration), # Girar 90 grados
                (0.15, 0.0, 2.0),           # Adelante
                (0.0, 0.5, angular_duration)  # Girar 90 grados
            ])

        # Posicionamiento para el 7
        self.get_logger().info('Trazo del 0 completado. Posicionando para el 7...')
        commands.extend([
            (0.0, -0.5, angular_duration), # Girar -90 grados (orientación para avanzar)
            (0.15, 0.0, 3.0),             # Avanzar un poco (separación)
            (0.0, 0.5, angular_duration)   # Girar 90 grados (orientación correcta)
        ])
        
        # Dibujando 7
        self.get_logger().info('Preparando comandos para dibujar 7...')
        commands.extend([
            (0.25, 0.0, 4.0),           # Adelante (largo superior)
            (0.0, 0.5, angular_duration), # Girar 90 grados
            (0.15, 0.0, 2.0)            # Adelante (diagonal o vertical)
        ])
        
        return commands

    
    def _control_loop(self):
        """Bucle de control principal ejecutado por el Timer cada 50ms."""
        
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 1. Lógica de Evitación (Prioridad Máxima)
        if self.obstacle_detected:
            # Si se está moviendo, publica la orden de parada
            if self.current_twist.linear.x != 0.0 or self.current_twist.angular.z != 0.0:
                self.stop()
            # No ejecuta la lógica de dibujo, simplemente publica la última orden (0,0)
            self.publisher.publish(self.current_twist)
            return
        
        # 2. Lógica de Secuencia de Movimiento (Solo si no hay obstáculo)
        if self.drawing_active and self.state == 'IDLE':
            try:
                # Obtener el siguiente comando de la secuencia
                lin_vel, ang_vel, duration = next(self.drawing_sequence)
                self.current_twist.linear.x = lin_vel
                self.current_twist.angular.z = ang_vel
                self.end_time = current_time + duration
                self.state = 'MOVING'
                self.get_logger().info(f' Nuevo comando: L={lin_vel:.2f}, A={ang_vel:.2f}, Dur={duration:.2f}s')

            except StopIteration:
                # Secuencia de dibujo completada
                self.drawing_active = False
                self.state = 'FINISHED'
                self.stop() # Asegura la parada final y el log
                self.get_logger().info(' Trazo del 07 completado.')
                return

        # 3. Lógica de Movimiento
        if self.state == 'MOVING':
            if current_time >= self.end_time:
                # Comando de movimiento ha terminado
                self.stop()
                self.state = 'IDLE' # Pasa a IDLE para cargar el siguiente comando
            else:
                # Continúa ejecutando el comando
                self.publisher.publish(self.current_twist)


    def stop(self):
        """Detiene el robot inmediatamente y actualiza el comando actual."""
        # Si ya está detenido, no hace nada
        if self.current_twist.linear.x == 0.0 and self.current_twist.angular.z == 0.0:
            return

        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher.publish(self.current_twist)
        self.get_logger().info(' Robot detenido.')


    def destroy(self):
        """Finaliza correctamente el nodo."""
        # Detener el robot antes de cerrar
        self.stop()
        self.get_logger().info('Finalizando nodo avoid_colisions...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AvoidColisions()
    try:
        # rclpy.spin() procesa todos los callbacks (suscripciones y timers) de forma concurrente
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()