import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time

class OdometryAckermann(Node):
    def __init__(self):
        super().__init__('odometry_ackermann')
        
        # Suscripción a los comandos que recibe el robot (v y delta)
        self.sub_cmd = self.create_subscription(Twist, 'turtlebot_cmdVel', self.update_odometry, 10)
        
        # Publicador de la posición calculada para la interfaz
        self.pub_pos = self.create_publisher(Twist, 'turtlebot_position', 10)

        # Parámetros físicos del robot (Ajustar según tu construcción real)
        self.L = 0.2  # Distancia entre ejes (Wheelbase) en metros 
        
        # Estado del robot (x, y, theta) en el marco global
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def update_odometry(self, msg):
        # 1. Obtener velocidades actuales
        v = msg.linear.x        # Velocidad lineal [cite: 241]
        delta = msg.angular.z   # Ángulo de dirección de las ruedas frontales 
        
        # 2. Calcular el diferencial de tiempo
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 3. Aplicar Modelo Cinemático Ackermann [cite: 242, 246]
        # x_dot = v * cos(theta)
        # y_dot = v * sin(theta)
        # theta_dot = (v / L) * tan(delta)
        
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += (v / self.L) * np.tan(delta) * dt

        # 4. Publicar la posición estimada para que la interfaz la grafique
        pos_msg = Twist()
        pos_msg.linear.x = self.x
        pos_msg.linear.y = self.y
        pos_msg.angular.z = self.theta
        self.pub_pos.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
