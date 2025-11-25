import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class EvadeColisions(Node):
    
    def __init__(self):
        super().__init__('evade_colisions')
        
        # --- Parámetros de Control ---
        # La distancia crítica de seguridad (ajusta este valor)
        self.DISTANCIA_PELIGRO = 0.35  
        
        # Velocidades
        self.VELOCIDAD_NORMAL = 0.15   # Velocidad lineal de avance (m/s)
        self.VELOCIDAD_GIRO = 0.5      # Velocidad angular para girar (rad/s)
        
        # --- Inicialización ---
        # Publisher para enviar comandos de velocidad
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber para recibir datos del sensor láser
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.get_logger().info('Nodo Evade Colisiones Inicializado con D_peligro: 0.35m')

    def laser_callback(self, msg):
        """
        Función de callback que procesa los datos de LaserScan y aplica la lógica de control.
        """
        
        # Filtrado de Datos del Lidar:
        # 1. Rango de visión: Tomamos los 90 grados frontales derechos y los 90 grados frontales izquierdos.
        rango_frente_derecho = msg.ranges[0:90]
        rango_frente_izquierdo = msg.ranges[-90:] 
        
        rango_frente = np.array(rango_frente_derecho + rango_frente_izquierdo)
        
        # Eliminar valores no válidos (infinitos y NaN) que son comunes en Lidar
        rango_valido = rango_frente[np.isfinite(rango_frente)]
        
        # Encontrar la distancia mínima
        if rango_valido.size == 0:
            min_distancia = 99.0 
        else:
            min_distancia = np.min(rango_valido)
        
        
        # --- Lógica de Control ---
        twist = Twist()

        if min_distancia < self.DISTANCIA_PELIGRO:
            # CASO DE PELIGRO: DETENERSE Y GIRAR
            
            # Detenemos el avance
            twist.linear.x = 0.0
            
            # Aplicamos un giro (por defecto a la izquierda)
            twist.angular.z = self.VELOCIDAD_GIRO 
            
            self.publisher_.publish(twist)
            self.get_logger().warn(f"Obstáculo detectado a {min_distancia:.2f}m. Evadiendo.")
            
        else:
            # CASO SEGURO: AVANZAR
            twist.linear.x = self.VELOCIDAD_NORMAL
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            # self.get_logger().info(f"Ruta despejada. Distancia mínima: {min_distancia:.2f}m")
            
# --- Función Principal para ROS 2 ---
def main(args=None):
    rclpy.init(args=args)
    node = EvadeColisions()
    
    # Mantiene el nodo activo para procesar los callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parada manual. Destruyendo nodo.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()