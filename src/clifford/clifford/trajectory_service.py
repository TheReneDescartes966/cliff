import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from custom_interfaces.srv import TrajectoryPoint
import numpy as np

class TrayectoriaServicio(Node):
    def __init__(self):
        super().__init__("trajectory_service")
        
        # Parámetros de la elipse
        # Recordar que todo esta siendo tomado en metros
        self.a = -0.04
        self.h = 0.015
        self.z0 = -0.158
        self.num_puntos = 10
        
        # Generar trayectoria
        self.t_vals = np.linspace(0, 50, self.num_puntos)
        self.x_trayectoria = np.round(self.a * (self.t_vals / 50) + 0.02, 3)
        self.z_trayectoria = np.round(self.z0 + self.h * np.sin(np.pi * (self.t_vals / 50)), 3)
        
        # Crear servicio
        self.srv = self.create_service(TrajectoryPoint, 'get_trajectory_point', self.handle_request)

    def handle_request(self, request, response):
        punto = Point()

        if request.index < self.num_puntos:
            punto.x = self.x_trayectoria[request.index]
            punto.z = self.z_trayectoria[request.index]
            #self.get_logger().warning(f"PUNTO X= {punto.x}")
            #self.get_logger().warning(f"PUNTO Y= {punto.y}")
            #self.get_logger().warning(f"PUNTO Z= {punto.z}")
        else:
            self.get_logger().warning("Índice fuera de rango")

        punto.y = 0.0 
        response.point = punto
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrayectoriaServicio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
