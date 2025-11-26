#!/usr/bin/env python3
import cv2
import numpy as np
from collections import Counter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoUniversalNode(Node):
    def __init__(self):
        super().__init__('aruco_universal_node')

        # Configuración del tópico (ajústalo si es necesario)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').value

        # --- 1. LISTA COMPLETA DE DICCIONARIOS ---
        # Reactivamos todos porque tus marcadores parecen ser 4x4 o 5x5 básicos
        self.ARUCO_DICTS = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }

        # --- 2. PARÁMETROS EQUILIBRADOS ---
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # UMBRALIZACIÓN (El ajuste clave para tus fotos)
        # Antes estaba en 23 (muy pequeño para tus marcadores grandes).
        # Lo subimos a 45. Si sigue fallando, prueba a subirlo a 55 o 65.
        self.parameters.adaptiveThreshWinSizeMin = 5
        self.parameters.adaptiveThreshWinSizeMax = 45 
        self.parameters.adaptiveThreshWinSizeStep = 10
        
        # FILTROS DE FORMA
        # Relajamos un poco la precisión poligonal para aceptar formas "pixeladas"
        self.parameters.polygonalApproxAccuracyRate = 0.06
        
        # FILTRO DE TAMAÑO (Para seguir ignorando ladrillos pequeños)
        self.parameters.minMarkerPerimeterRate = 0.03 

        # --- Variables del sistema ---
        self.current_dict_obj = None
        self.current_dict_name = None
        self.is_locked = False 
        self.br = CvBridge()
        
        # Suscriptor y Publicadores
        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/camera/aruco_result', 10)
        # Tópico DEBUG: Suscríbete aquí en RViz para ver qué "ve" el robot
        self.pub_debug = self.create_publisher(Image, '/camera/debug_view', 10)

        self.get_logger().info('✅ Nodo Universal Iniciado: Parámetros ajustados para bloques grandes.')

    def buscar_mejor_diccionario(self, gray_frame):
        """Busca en todos los diccionarios hasta encontrar uno válido."""
        votos = []
        for nombre_dict, codigo_dict in self.ARUCO_DICTS.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(codigo_dict)
            corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=self.parameters)

            if ids is not None:
                for i in range(len(ids)):
                    # Filtramos cosas minúsculas (probablemente ruido de ladrillos)
                    perimetro = cv2.arcLength(corners[i][0], True)
                    if perimetro > 80: 
                        votos.append({"id": ids[i][0], "dict_name": nombre_dict})

        if not votos: return None, None

        conteo = Counter([v["id"] for v in votos])
        id_ganador, _ = conteo.most_common(1)[0]
        
        # Obtenemos el nombre del diccionario ganador
        nombre_ganador = next(v["dict_name"] for v in votos if v["id"] == id_ganador)
        return nombre_ganador, self.ARUCO_DICTS[nombre_ganador]

    def image_callback(self, data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError: return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # --- DEBUG VISUAL (Crucial para diagnóstico) ---
        # Publicamos la imagen binaria que usa OpenCV internamente
        debug_img = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 
                                          self.parameters.adaptiveThreshWinSizeMax, 
                                          self.parameters.adaptiveThreshConstant)
        self.pub_debug.publish(self.br.cv2_to_imgmsg(debug_img, "mono8"))
        # -----------------------------------------------

        # Lógica de detección
        if not self.is_locked:
            nombre, codigo = self.buscar_mejor_diccionario(gray)
            if nombre:
                self.current_dict_name = nombre
                self.current_dict_obj = cv2.aruco.getPredefinedDictionary(codigo)
                self.is_locked = True
                self.get_logger().info(f"🔓 DICCIONARIO ENCONTRADO: {nombre}")
            else:
                # Feedback en pantalla mientras busca
                cv2.putText(cv_image, "Escaneando...", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            # Detección rápida una vez conocemos el diccionario
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.current_dict_obj, parameters=self.parameters)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                # Texto verde confirmando detección
                cv2.putText(cv_image, f"ID: {ids.flatten()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                self.get_logger().info(f"Detectado: {ids.flatten()}")
            else:
                cv2.putText(cv_image, "Perdido...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                # Opcional: Desbloquear si se pierde mucho tiempo para buscar otro diccionario
                # self.is_locked = False 

        # Publicar resultado final en color
        self.publisher.publish(self.br.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoUniversalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
