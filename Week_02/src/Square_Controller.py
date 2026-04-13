import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # --- PARÁMETROS ---
        self.side_length = 2.0  # metros
        self.target_speed = 0.3  # m/s  
        self.angular_speed = 2.5 # rad/s
        
        # --- AUTO-TUNE: Estimación de tiempos  ---
        self.t_straight = self.side_length / self.target_speed
        self.t_turn = (np.pi / 2) / self.angular_speed # Giro de 90 grados
        
        # --- ESTADOS ---
        self.state = 'STRAIGHT'
        self.count = 0  # Contador de vueltas
        self.state_start_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Challenge 1: Square Path Initialized')

    def control_L(self, linear_speed):
        k = 1.05
        return linear_speed / k

    def control_A(self, angular_speed):
        if angular_speed < 0.6:
            k = 1.2
        elif angular_speed < 2.0:
            k = 1.06
        else:
            k = 1.08
        return angular_speed / k

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        if self.state == 'STRAIGHT':
            cmd.linear.x = self.control_L(self.target_speed)
            if elapsed >= self.t_straight:
                self.state = 'TURN'
                self.state_start_time = now
                self.get_logger().info(f'Side {self.count + 1} done. Turning...')

        elif self.state == 'TURN':
            cmd.angular.z = self.control_A(self.angular_speed)
            if elapsed >= self.t_turn:
                self.count += 1
                if self.count >= 4: # Ya hizo los 4 lados
                    self.state = 'STOP'
                else:
                    self.state = 'STRAIGHT'
                self.state_start_time = now

        elif self.state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Square completed!')
            self.timer.cancel()

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()