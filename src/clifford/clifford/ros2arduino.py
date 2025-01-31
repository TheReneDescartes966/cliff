import rclpy
import serial 
import math
import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState 
from rclpy.node import Node

class ros2arduino(Node):
    def __init__(self):
        super().__init__("transform_angles_ros2arduino")
        self.position_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_states_callback, 
            10
        )
        self.id = [5,4,2,3]
        self.cont = 0
        self.cont_id = 0
        self.cont_angulos = 0

        # Serial port's information
        self.microcontroller_port = "/dev/ttyUSB0"
        self.microcontroller_baudrate = 115200
        self.serial_port = serial.Serial(
            self.microcontroller_port, 
            self.microcontroller_baudrate, 
            timeout=10
        )
        self.mapped_msg = String()
        


    def map_position_into_bits(self, value, in_min, in_max, out_min, out_max):
        mapped_data_list = (value - in_min) * (out_max - out_min) / ((in_max - in_min) + out_min)
        return round(mapped_data_list)
         
    def write_into_serial_port(self, mapped_data_list):
        self.cont += 1
        if self.cont >= 5:
            self.cont = 0
            self.cont_angulos += 3
            if self.cont_angulos >= len(mapped_data_list):
                self.cont_angulos = 0

            self.cont_id += 1
            if self.cont_id >= len(self.id):
                self.cont_id = 0

        data_out = f"{self.id[self.cont_id]}:c{mapped_data_list[self.cont_angulos]},{mapped_data_list[self.cont_angulos + 1]},{mapped_data_list[self.cont_angulos + 2]}"
        self.get_logger().info(data_out)

        # Enviar al puerto serial
        self.serial_port.write(data_out.encode())


        

        """if theta3 <= np.radians(-92):
                    theta3 = np.radians(-92)

                if theta3 >= np.radians(26):
                    theta3 = np.radians(26)

                if theta5 <= np.radians(-107):
                    theta5 = np.radians(-107)            

                if theta5 >= np.radians(41):
                    theta5 = np.radians(40)"""
            
        
    def joint_states_callback(self, joint_states_msg):
        print("JointState received!")

        front_right_q1_in_degrees = int(math.degrees(joint_states_msg.position[0]))
        front_right_q2_in_degrees = int(math.degrees(joint_states_msg.position[1]))
        front_right_q3_in_degrees = int(math.degrees(joint_states_msg.position[2]))

        front_left_q1_in_degrees = int(math.degrees(joint_states_msg.position[4]))
        front_left_q2_in_degrees = int(math.degrees(joint_states_msg.position[5]))
        front_left_q3_in_degrees = int(math.degrees(joint_states_msg.position[6]))

        back_right_q1_in_degrees = int(math.degrees(joint_states_msg.position[8]))
        back_right_q2_in_degrees = int(math.degrees(joint_states_msg.position[9]))
        back_right_q3_in_degrees = int(math.degrees(joint_states_msg.position[10]))

        back_left_q1_in_degrees = int(math.degrees(joint_states_msg.position[12]))
        back_left_q2_in_degrees = int(math.degrees(joint_states_msg.position[13]))
        back_left_q3_in_degrees = int(math.degrees(joint_states_msg.position[14]))

        front_right_q1_in_bits = self.map_position_into_bits(front_right_q1_in_degrees , -150, 150, 0, 1023)
        front_right_q2_in_bits = self.map_position_into_bits(front_right_q2_in_degrees , 150, -150, 0, 1023)
        front_right_q3_in_bits = self.map_position_into_bits(front_right_q3_in_degrees , -150, 150, 0, 1023)

        front_left_q1_in_bits = self.map_position_into_bits(front_left_q1_in_degrees , -150, 150, 0, 1023)
        front_left_q2_in_bits = self.map_position_into_bits(front_left_q2_in_degrees , 150, -150, 0, 1023)
        front_left_q3_in_bits = self.map_position_into_bits(front_left_q3_in_degrees , -150, 150, 0, 1023)

        back_right_q1_in_bits = self.map_position_into_bits(back_right_q1_in_degrees , -150, 150, 0, 1023)
        back_right_q2_in_bits = self.map_position_into_bits(back_right_q2_in_degrees , 150, -150, 0, 1023)
        back_right_q3_in_bits = self.map_position_into_bits(back_right_q3_in_degrees , -150, 150, 0, 1023)

        back_left_q1_in_bits = self.map_position_into_bits(back_left_q1_in_degrees , -150, 150, 0, 1023)
        back_left_q2_in_bits = self.map_position_into_bits(back_left_q2_in_degrees , 150, -150, 0, 1023)
        back_left_q3_in_bits = self.map_position_into_bits(back_left_q3_in_degrees , -150, 150, 0, 1023)


        mapped_data_list = [
            front_right_q1_in_degrees,
            front_right_q2_in_degrees,
            front_right_q3_in_degrees,
            front_left_q1_in_degrees,
            front_left_q2_in_degrees,
            front_left_q3_in_degrees,
            back_right_q1_in_degrees,
            back_right_q2_in_degrees,
            back_right_q3_in_degrees,
            back_left_q1_in_degrees,
            back_left_q2_in_degrees,
            back_left_q3_in_degrees            
        ]
        

        self.get_logger().info(f"VALOR PATA DERECHA ADELATE: \n {mapped_data_list[0]}, {mapped_data_list[1]}, {mapped_data_list[2]}")
        self.get_logger().info(f"VALOR PATA IZQUIERDA ADELATE: \n {mapped_data_list[3]}, {mapped_data_list[4]}, {mapped_data_list[5]}")
        self.get_logger().info(f"VALOR PATA DERECHA ATRAS: \n {mapped_data_list[6]}, {mapped_data_list[7]}, {mapped_data_list[8]}")
        self.get_logger().info(f"VALOR PATA IZQUIERDA ATRAS: \n {mapped_data_list[9]}, {mapped_data_list[10]}, {mapped_data_list[11]}")


        self.write_into_serial_port(mapped_data_list)


  
def main(args=None):
    rclpy.init(args=args)
    node = ros2arduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    