import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopDiferencial(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # Cambiado al tópico estándar del robot real
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        print("--- Configuración Robot Diferencial ---")
        self.v_max = float(input("Velocidad lineal máxima (m/s): "))
        self.w_max = float(input("Velocidad angular máxima (rad/s): "))
        
        self.twist = Twist()

    def on_press(self, key):
        try:
            if key.char == 'w': self.twist.linear.x = self.v_max
            elif key.char == 's': self.twist.linear.x = -self.v_max
            elif key.char == 'a': self.twist.angular.z = self.w_max
            elif key.char == 'd': self.twist.angular.z = -self.w_max
            self.publisher.publish(self.twist)
        except AttributeError: pass

    def on_release(self, key):
        try:
            if key.char in ['w', 's']: self.twist.linear.x = 0.0
            if key.char in ['a', 'd']: self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
        except AttributeError: pass

def main():
    rclpy.init()
    node = TeleopDiferencial()
    with keyboard.Listener(on_press=node.on_press, on_release=node.on_release) as listener:
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
