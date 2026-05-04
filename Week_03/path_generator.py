import rclpy
import numpy as np
import transforms3d

from rclpy.node import Node
from nav_msgs.msg import Odometry
from robloxianas_msgs.msg import PosePose

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # 1. Lista de objetivos (x, y)
        self.targets = [
             {'x': 2.0, 'y': 0.0},
             {'x': 2.0, 'y': 2.0},
             {'x': 0.0, 'y': 2.0},
             {'x': 0.0, 'y': 0.0}
        ]
        
        # Parámetros dinámicos
        self.max_linear_vel = 0.4  # m/s
        self.max_angular_vel = 4.0  # rad/s
        self.dist_tolerance = 0.10  # 10 cm
        self.angle_tolerance = 0.08 # ~4.5 grados

        # Estado interno
        self.current_target_idx = 0
        self.sub_step = "turn" 
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.last_logged_step = None

        # Pub/Sub
        self.goal_pub = self.create_publisher(PosePose, 'goals', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.01, self.publish_next_goal) # 10Hz para suavidad
        self.get_logger().info("Supervisor de Trayectorias iniciado.")

    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_pose['theta'] = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def publish_next_goal(self):
        if self.current_target_idx < len(self.targets):
            target = self.targets[self.current_target_idx]
            msg = PosePose()
            
            # Cálculo de errores
            dx = target['x'] - self.robot_pose['x']
            dy = target['y'] - self.robot_pose['y']
            dist = np.sqrt(dx**2 + dy**2)
            target_angle = np.arctan2(dy, dx)
            
            # Error de ángulo normalizado (-pi a pi)
            error_theta = np.arctan2(np.sin(target_angle - self.robot_pose['theta']), 
                                     np.cos(target_angle - self.robot_pose['theta']))

            if self.sub_step == "turn":
                msg.move_type = "turn"
                # Control Proporcional para giro suave
                msg.velocity = float(np.clip(2.0 * error_theta, -self.max_angular_vel, self.max_angular_vel))
                
                if abs(error_theta) < self.angle_tolerance:
                    self.get_logger().info(f"--> Giro completado. Avanzando a punto {self.current_target_idx + 1}")
                    self.sub_step = "advance"

            elif self.sub_step == "advance":
                msg.move_type = "advance"
                # Control Proporcional para frenado suave al llegar
                msg.velocity = float(np.clip(0.8 * dist, -self.max_linear_vel, self.max_linear_vel))
                
                if dist < self.dist_tolerance:
                    self.get_logger().info(f"✔ Punto {self.current_target_idx + 1} alcanzado.")
                    self.current_target_idx += 1
                    self.sub_step = "turn"

            # Datos adicionales del mensaje
            msg.is_reachable = True # Aquí podrías añadir tu lógica dinámica
            self.goal_pub.publish(msg)
            
        else:
            # Enviar comando de parada final
            stop_msg = PosePose()
            stop_msg.move_type = "stop"
            stop_msg.velocity = 0.0
            self.goal_pub.publish(stop_msg)
            if self.last_logged_step != "FINISHED":
                self.get_logger().info("--- ¡MISIÓN CUMPLIDA! ---")
                self.last_logged_step = "FINISHED"

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()