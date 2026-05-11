import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robloxianas_msgs.msg import TargetPose

class ControlLazoCerrado(Node):
    def __init__(self):
        super().__init__('lazo_cerrado_node')

        # Suscripción actualizada a TargetPose
        self.goal_sub = self.create_subscription(TargetPose, 'goals', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variables de estado
        self.v_real = 0.0
        self.w_real = 0.0
        self.v_ref = 0.0
        self.w_ref = 0.0

        # Parámetros PI
        self.dt = 0.01
        self.Kp_v, self.Ki_v = 1.1, 0.35
        self.Kp_w, self.Ki_w = 1.3, 0.35

        self.e_v_int, self.e_w_int = 0.0, 0.0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Controlador PI Dual (TargetPose) activo.")

    def odom_callback(self, msg):
        self.v_real = msg.twist.twist.linear.x
        self.w_real = msg.twist.twist.angular.z

    def goal_callback(self, msg):
        self.v_ref = msg.velocity_lin
        self.w_ref = msg.velocity_ang
        
        if self.v_ref == 0.0 and self.w_ref == 0.0:
            self.e_v_int = 0.0
            self.e_w_int = 0.0

    def control_linear(self, v_ref):
        e = v_ref - self.v_real
        self.e_v_int += e * self.dt
        self.e_v_int = np.clip(self.e_v_int, -0.5, 0.5) # Anti-windup
        u = self.Kp_v * e + self.Ki_v * self.e_v_int
        return float(np.clip(u, -0.4, 0.4))

    def control_angular(self, w_ref):
        e = w_ref - self.w_real
        self.e_w_int += e * self.dt
        self.e_w_int = np.clip(self.e_w_int, -0.5, 0.5) # Anti-windup
        u = self.Kp_w * e + self.Ki_w * self.e_w_int
        return float(np.clip(u, -4.0, 4.0))

    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = self.control_linear(self.v_ref)
        cmd.angular.z = self.control_angular(self.w_ref)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlLazoCerrado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido por usuario")
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()