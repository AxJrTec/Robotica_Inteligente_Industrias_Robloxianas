import rclpy
import numpy as np
import transforms3d

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from robloxianas_msgs.msg import TargetPose 

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # 1. Lista de objetivos (x, y)
        self.targets = [
             {'x': 1.0, 'y': 0.0},
             {'x': 1.0, 'y': 1.0},
             {'x': 0.0, 'y': 1.0},
             {'x': 0.0, 'y': 0.0}
        ]
        
        # Parámetros de límites físicos
        self.max_linear_vel = 0.4
        self.max_angular_vel = 4.0
        self.dist_tolerance = 0.05

        # Lógica de Semáforo
        self.semaforo = 'VERDE'
        self.speed_factor = 1.0 

        # Estado interno
        self.current_target_idx = 0
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.mission_finished = False

        # Pub/Sub
        self.goal_pub = self.create_publisher(TargetPose, 'goals', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.semf_sub = self.create_subscription(String, 'estado_semaforo', self.semf_callback, 10)

        # Timer a 100Hz
        self.timer = self.create_timer(0.01, self.publish_next_goal)

        self.get_logger().info("Generador de Trayectorias iniciado")
        self.get_logger().info(f"Total de puntos: {len(self.targets)}")

    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_pose['theta'] = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def semf_callback(self, msg):
        self.semaforo = msg.data.upper()
        if self.semaforo == 'VERDE':
            self.speed_factor = 1.0
        elif self.semaforo == 'AMARILLO':
            self.speed_factor = 0.2
        elif self.semaforo == 'ROJO':
            self.speed_factor = 0.0
        else:
            self.speed_factor = 1.0

    def publish_next_goal(self):
        if self.current_target_idx >= len(self.targets):
            if not self.mission_finished:
                self.stop_robot()
                self.get_logger().info("¡TRAYECTORIA COMPLETADA!")
                self.mission_finished = True
            return

        if self.speed_factor == 0.0:
            self.stop_robot()
            return

        target = self.targets[self.current_target_idx]
        msg = TargetPose()
        
        msg.current_goal = Pose()
        msg.current_goal.position.x = float(target['x'])
        msg.current_goal.position.y = float(target['y'])
        
        msg.next_goal = Pose()
        if self.current_target_idx + 1 < len(self.targets):
            nt = self.targets[self.current_target_idx + 1]
            msg.next_goal.position.x = float(nt['x'])
            msg.next_goal.position.y = float(nt['y'])
        else:
            msg.next_goal = msg.current_goal

        dx = target['x'] - self.robot_pose['x']
        dy = target['y'] - self.robot_pose['y']
        dist = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)

        error_theta = np.arctan2(
            np.sin(target_angle - self.robot_pose['theta']), 
            np.cos(target_angle - self.robot_pose['theta'])
        )

        if dist < self.dist_tolerance:
            self.get_logger().info(f"Punto {self.current_target_idx + 1} alcanzado")
            self.current_target_idx += 1
            return

        k_linear, k_angular = 0.8, 2.0

        if abs(error_theta) > 0.52:
            msg.velocity_lin = 0.0
            msg.velocity_ang = (k_angular * error_theta) * self.speed_factor
        else:
            msg.velocity_lin = (k_linear * dist) * self.speed_factor
            msg.velocity_ang = (k_angular * error_theta) * self.speed_factor

        msg.velocity_lin = np.clip(msg.velocity_lin, -self.max_linear_vel, self.max_linear_vel)
        msg.velocity_ang = np.clip(msg.velocity_ang, -self.max_angular_vel, self.max_angular_vel)

        msg.is_reachable = True
        self.goal_pub.publish(msg)

    def stop_robot(self):
        self.get_logger().info("Deteniendo robot")
        stop_msg = TargetPose()
        stop_msg.velocity_lin = 0.0
        stop_msg.velocity_ang = 0.0
        stop_msg.is_reachable = True
        self.goal_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido por usuario")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()