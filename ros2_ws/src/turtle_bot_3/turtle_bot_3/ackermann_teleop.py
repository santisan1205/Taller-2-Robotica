import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopAckermann(Node):
    def __init__(self):
        super().__init__('teleop_ackermann')
        self.publisher = self.create_publisher(Twist, 'turtlebot_cmdVel', 10)
        
        # Parámetros iniciales solicitados al usuario (Requerimiento Guía)
        print("--- Configuración de Teleoperación Ackermann ---")
        self.v_max = float(input("Ingrese velocidad lineal máxima (m/s): "))
        self.delta_max = float(input("Ingrese ángulo de dirección máximo (rad): "))
        
        self.twist = Twist()
        self.get_logger().info(f'Nodo Teleop listo. W/S: Velocidad, A/D: Dirección. v_max: {self.v_max}, delta_max: {self.delta_max}')

        # Listener de teclado
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.twist.linear.x = self.v_max
            elif key.char == 's':
                self.twist.linear.x = -self.v_max
            elif key.char == 'a':
                self.twist.angular.z = self.delta_max
            elif key.char == 'd':
                self.twist.angular.z = -self.delta_max
            
            self.publisher.publish(self.twist)
        except AttributeError:
            pass

    def on_release(self, key):
        # Lógica de seguridad: velocidad nula si no hay pulsación (Criterio 10)
        try:
            if key.char in ['w', 's']:
                self.twist.linear.x = 0.0
            if key.char in ['a', 'd']:
                self.twist.angular.z = 0.0
            
            self.publisher.publish(self.twist)
        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TeleopAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
