import rclpy
import numpy as np
import pandas as pd
import time
from sensor_msgs.msg import JointState
from rclpy.node import Node
from clifford.numeric_method import nodeNumericMethod
from custom_interfaces.srv import TrajectoryPoint

class QuadrupedLocomotion(Node):
    def __init__(self):
        super().__init__("quadruped_locomotion")

        # Parámetros generales
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0

        self.q_pata_da = [0.0, 0.0, 0.0, 0.0]
        self.q_pata_ia = [0.0, 0.0, 0.0, 0.0]
        self.q_pata_dt = [0.0, 0.0, 0.0, 0.0]
        self.q_pata_it = [0.0, 0.0, 0.0, 0.0]

        self.joint_names = [
            "hombro_DA_joint", "brazo_DA_joint", "muneca_DA_joint", "end_effector_DA_joint",
            "hombro_IA_joint", "brazo_IA_joint", "muneca_IA_joint", "end_effector_IA_joint",
            "hombro_DT_joint", "brazo_DT_joint", "muneca_DT_joint", "end_effector_DT_joint",
            "hombro_IT_joint", "brazo_IT_joint", "muneca_IT_joint", "end_effector_IT_joint"
        ]
        
        self.l_list = [0.052, 0.041, 0.090] #Eslabones para el uso de la geometrica

        self.current_leg = 0 #Numero para indicar la pata a mover
        self.num_points = 30 #Numero de puntos de la elipse
        self.reverse_phase = False #Bandera para la fase de retorno

        # Parámetros de la elipse
        self.a = -0.020
        self.h = 0.010
        self.z0 = -0.162

        # Publisher de /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Método numérico
        self.numeric_method = nodeNumericMethod()

        # Cliente para el servicio de trayectoria
        self.cli = self.create_client(TrajectoryPoint, 'get_trajectory_point')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio "get_trajectory_point"...')
        self.get_logger().info('Servicio "get_trajectory_point" conectado.')

        # Índice para la trayectoria
        self.index = 0

    def selec_move_leg(self, current_leg):
        punto = self.send_request(self.index)
        
        self.px = punto.x
        self.py = punto.y
        self.pz = punto.z

        desired_point = [self.px, self.py, self.pz, 0.0]
        self.q_list = self.numeric_method.numeric_method(desired_point)

        self.value_in_joint = [*self.q_pata_da, 
                              *self.q_pata_ia,
                              *self.q_pata_dt,
                              *self.q_pata_it]

        if current_leg == 0:
            #Aqui se debe escoger la pata_IT
            self.q_pata_it = -1*(self.q_list)
            self.move_leg()
        
        elif current_leg == 1:
            #Aqui se debe escoger la pata_DT
            self.q_pata_dt = self.q_list
            self.move_leg()
            
        elif current_leg == 2:
            #Aqui se debe escoger la pata_DA
            self.q_pata_da = self.q_list
            self.move_leg()
            
        elif current_leg == 3:
            #Aqui se debe escoger la pata_IA
            self.q_pata_ia = -1*(self.q_list)
            self.move_leg()

    
    def move_leg(self):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint_names
        joint_states.position = self.value_in_joint
        self.publisher_.publish(joint_states)
    
    def send_request(self, index):
        req = TrajectoryPoint.Request()
        req.index = index

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().point
        else:
            self.get_logger().error('Error al llamar al servicio')
            return None
    def execute_locomotion(self):
        try:
            while rclpy.ok():
                # Fase 1: Todas las patas siguen la trayectoria una por una
                if self.index < self.num_points:
                    self.selec_move_leg(self.current_leg)
                    self.index += 1
                else:
                    # Resetear índice para la siguiente pata
                    self.index = 0

                    # Si hemos movido la última pata (IT), pasar a la fase de retorno a posición inicial
                    if self.current_leg == 3:
                        self.return_to_home_phase = True
                        self.current_leg = 0
                    else:
                        self.current_leg += 1

                # Fase 2: Enviar cada pata de regreso a la posición inicial
                if hasattr(self, 'return_to_home_phase') and self.return_to_home_phase:
                    if self.index < self.num_points:
                        # Mover la pata actual a su posición inicial
                        #punto = self.send_request(0)  # Posición inicial
                        desired_point = [0.04, 0, -0.162, 0.0]
                        self.q_list = self.numeric_method.numeric_method(desired_point)

                        # Aplicar el movimiento a la pata actual
                        if self.current_leg == 0:
                            self.q_pata_da = self.q_list
                        elif self.current_leg == 1:
                            self.q_pata_ia = self.q_list
                        elif self.current_leg == 2:
                            self.q_pata_dt = self.q_list
                        elif self.current_leg == 3:
                            self.q_pata_it = self.q_list

                        self.move_leg()
                        self.index += 1
                    else:
                        # Resetear para el próximo ciclo
                        self.index = 0
                        self.current_leg += 1

                        # Si todas las patas volvieron a HOME, reiniciar ciclo
                        if self.current_leg > 3:
                            self.current_leg = 0
                            self.return_to_home_phase = False

                # Pequeña pausa para visualización
                time.sleep(0.01)
        except Exception as e:
            self.get_logger().error(f'Error en la locomoción: {str(e)}')

    """def execute_locomotion(self):
        try:
            while rclpy.ok():
                # Si estamos en la fase normal (no retorno)
                if not self.reverse_phase:
                    if self.index < self.num_points:
                        self.selec_move_leg(self.current_leg)
                        self.index += 1
                    else:
                        # Resetear índice para la siguiente pata
                        self.index = 0
                        
                        # Si estamos en la última pata (IT)
                        if self.current_leg == 3:
                            self.reverse_phase = True
                            self.current_leg = 0
                        else:
                            self.current_leg += 1
                            
                # Fase de retorno (solo para las primeras 3 patas)
                else:
                    if self.index < self.num_points:
                        # Mover la pata IT normalmente
                        self.selec_move_leg(3)
                        
                        # Mover las otras patas en reversa
                        punto = self.send_request(self.num_points - self.index - 1)
                        
                        # Actualizar posiciones para las otras patas
                        desired_point = [punto.x, punto.y, punto.z, 0.0]
                        self.q_list = self.numeric_method.numeric_method(desired_point)
                        
                        # Actualizar las primeras 3 patas
                        self.q_pata_da = self.q_list
                        self.q_pata_ia = -1*(self.q_list)
                        self.q_pata_dt = self.q_list
                        
                        self.move_leg()
                        self.index += 1
                    else:
                        # Resetear para el próximo ciclo
                        self.index = 0
                        self.current_leg = 0
                        self.reverse_phase = False
                        
                # Pequeña pausa para visualización
                time.sleep(0.01)"""
                
            


def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedLocomotion()
    try:
        node.execute_locomotion()
    except KeyboardInterrupt:
        node.get_logger().info('Locomoción interrumpida por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()