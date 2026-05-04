import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robloxianas_msgs.msg import PosePose 

class ControlLazoCerrado(Node):
    def __init__(self):
        super().__init__('control_lazo_cerrado_node')

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PosePose, 'goals', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variables de estado
        self.v_real = 0.0
        self.w_real = 0.0
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.current_move_type = ""

        # Parámetros PI (Ajustar según respuesta del Puzzlebot)
        self.dt = 0.01
        self.Kp_v, self.Ki_v = 1.1, 0.2
        self.Kp_w, self.Ki_w = 1.1, 0.2
        self.e_v_int, self.e_w_int = 0.0, 0.0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Controlador PI de Velocidad listo.")

    def odom_callback(self, msg):
        self.v_real = msg.twist.twist.linear.x
        self.w_real = msg.twist.twist.angular.z

    def goal_callback(self, msg):
        if msg.move_type == "stop":
            self.v_ref, self.w_ref = 0.0, 0.0
            self.current_move_type = "stop"
            return

        # Reset de integrales si cambia de fase para evitar "patadas"
        if msg.move_type != self.current_move_type:
            self.e_v_int = 0.0
            self.e_w_int = 0.0
            self.current_move_type = msg.move_type

        if msg.move_type == "turn":
            self.v_ref = 0.0
            self.w_ref = msg.velocity

        elif msg.move_type == "advance":
            self.v_ref = msg.velocity
            self.w_ref = 0.0

    def control_linear(self, v_ref):
        e = v_ref - self.v_real
        self.e_v_int += e * self.dt
        # Limitar integral (Anti-windup simple)
        self.e_v_int = np.clip(self.e_v_int, -0.5, 0.5)
        u = self.Kp_v * e + self.Ki_v * self.e_v_int
        return float(np.clip(u, -0.4, 0.4))

    def control_angular(self, w_ref):
        e = w_ref - self.w_real
        self.e_w_int += e * self.dt
        # Limitar integral (Anti-windup simple)
        self.e_w_int = np.clip(self.e_w_int, -0.5, 0.5)
        u = self.Kp_w * e + self.Ki_w * self.e_w_int
        return float(np.clip(u, -4.0, 4.0))

    def control_loop(self):
        cmd = Twist()
        
        # Si el supervisor dice STOP o no hay nada, mandamos 0
        if self.current_move_type == "stop" or self.current_move_type == "":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = self.control_linear(self.v_ref)
            cmd.angular.z = self.control_angular(self.w_ref)

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlLazoCerrado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()