from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. El Cerebro (Hardware real)
        Node(
            package='diff_bot_3',
            executable='robot_core',
            name='robot_core_node',
            output='screen'
        ),
        # 2. El Reproductor (Servicio)
        Node(
            package='diff_bot_3',
            executable='robot_player',
            name='player_diferencial',
            output='screen'
        ),
        # 3. La Interfaz Gráfica
        #Node(
        #    package='diff_bot_3',
        #    executable='robot_interface',
        #    name='robot_interface_node',
        #    output='screen',
        #    emulate_tty=True,
        #    arguments=['--ros-args', '--log-level', 'info'],
        #    on_exit=None,
            # -------------------
        #),
        # 4. El Control por Teclado
        #Node(
        #    package='diff_bot_3',
        #    executable='robot_teleop',
        #    name='teleop_node',
        #    output='screen',
        #    emulate_tty=True # Crucial para leer el teclado con pynput
        #)
    ])

