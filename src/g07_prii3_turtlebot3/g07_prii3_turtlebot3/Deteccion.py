#!/usr/bin/env python3
import cv2
import numpy as np
from collections import Counter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoRobustNode(Node):
    def __init__(self):
        super().__init__('aruco_robust_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').value

        # --- DICCIONARIOS ---
        # Reducimos la lista para evitar falsos positivos de diccionarios "gigantes"
        # Si sabes que son 4x4 o 5x5, es mejor comentar el resto.
        self.ARUCO_DICTS = {
           "DICT_7X7_50": cv2.aruco.DICT_7X7_50
        }

        # --- PARÁMETROS ROBUSTOS (LA CLAVE DEL ARREGLO) ---
        # Usamos sintaxis compatible con OpenCV 4.2 / 4.6
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # 1. TAMAÑO MÍNIMO: Ignorar cosas pequeñas (ladrillos)
        # El marcador debe ocupar al menos el 3% del perímetro de la imagen
        self.parameters.minMarkerPerimeterRate = 0.03 
        
        # 2. CALIDAD DEL CUADRADO: Ser exigente con la forma
        self.parameters.polygonalApproxAccuracyRate = 0.05
        
        # 3. UMBRALIZACIÓN: Ajustes para iluminación de simuladores
        # step size más grande ayuda a filtrar ruido fino
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        
        # 4. RECHAZO DE BORDES: Si el aruco toca el borde de la imagen, ¿lo aceptamos?
        # A veces ayuda ponerlo en 0 si la cámara se mueve mucho
        self.parameters.minDistanceToBorder = 3

        # Variables
        self.current_dict_obj = None
        self.current_dict_name = None
        self.is_locked = False 
        self.br = CvBridge()
        
        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/camera/aruco_filtered', 10)
        self.pub_debug = self.create_publisher(Image, '/camera/debug_threshold', 10) # NUEVO: Para ver qué ve el robot

        self.get_logger().info('Nodo Robusto Iniciado: Filtros de textura activados.')

    def buscar_mejor_diccionario(self, gray_frame):
        votos = []
        for nombre_dict, codigo_dict in self.ARUCO_DICTS.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(codigo_dict)
            corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=self.parameters)

            if ids is not None:
                for i in range(len(ids)):
                    # FILTRO EXTRA: Si detectamos algo, verificamos su tamaño
                    perimetro = cv2.arcLength(corners[i][0], True)
                    # Si es ridículamente pequeño (ladrillo lejano), no votamos por él
                    if perimetro > 100: 
                        votos.append({"id": ids[i][0], "dict_name": nombre_dict})

        if not votos: return None, None

        conteo = Counter([v["id"] for v in votos])
        id_ganador, num_votos = conteo.most_common(1)[0]
        
        # Si el ganador tiene muy pocos votos (ej. 1), podría ser ruido.
        # Exigimos consistencia (opcional, pero recomendado)
        if num_votos < 1: 
            return None, None

        nombre_ganador = next(v["dict_name"] for v in votos if v["id"] == id_ganador)
        return nombre_ganador, self.ARUCO_DICTS[nombre_ganador]

    def image_callback(self, data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError: return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # --- DEBUG VISUAL (IMPORTANTE) ---
        # Publicamos lo que el robot "ve" en blanco y negro. 
        # Si aquí ves los ladrillos muy marcados, hay que ajustar 'adaptiveThresh'
        debug_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 
                                             self.parameters.adaptiveThreshWinSizeMax, 
                                             self.parameters.adaptiveThreshConstant)
        self.pub_debug.publish(self.br.cv2_to_imgmsg(debug_thresh, "mono8"))
        # ---------------------------------

        if not self.is_locked:
            nombre, codigo = self.buscar_mejor_diccionario(gray)
            if nombre:
                self.current_dict_name = nombre
                self.current_dict_obj = cv2.aruco.getPredefinedDictionary(codigo)
                self.is_locked = True
                self.get_logger().info(f"DICCIONARIO DETECTADO: {nombre}")
            else:
                cv2.putText(cv_image, "Filtrando ruido...", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.current_dict_obj, parameters=self.parameters)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f"📍 Detectado: {ids.flatten()}")
                cv2.putText(cv_image, f"Dict: {self.current_dict_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                 # Si perdemos el tracking por mucho tiempo, podríamos desbloquear
                 pass

        self.publisher.publish(self.br.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoRobustNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
