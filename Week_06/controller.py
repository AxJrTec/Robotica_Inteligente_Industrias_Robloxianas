import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Controller(Node):
    def __init__(self):
        super().__init__('lazo_cerrado_node')

        # Suscripctor & Publisher
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.error_sub = self.create_subscription(Float32, 'error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variables de estado
        self.v_real = 0.0
        self.w_real = 0.0

        self.v_ref = 0.1 # m/s 
        self.Kp_vision = 0.015
        self.error_pixeles = 0.0

        # Parámetros PI
        self.dt = 0.02
        self.Kp_v, self.Ki_v = 1.1, 0.35
        self.Kp_w, self.Ki_w = 1.3, 0.35
        self.e_v_int, self.e_w_int = 0.0, 0.0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Controlador iniciado")

    def odom_callback(self, msg):
        self.v_real = msg.twist.twist.linear.x
        self.w_real = msg.twist.twist.angular.z

    def error_callback(self, msg):
        self.error_pixeles = msg.data
        self.last_error_time = self.get_clock().now()

    def control_linear(self, v_ref):
        if v_ref == 0.0:
            self.e_v_int = 0.0
            return 0.0

        e = v_ref - self.v_real
        self.e_v_int += e * self.dt
        self.e_v_int = np.clip(self.e_v_int, -0.5, 0.5) # Anti-windup
        
        u = self.Kp_v * e + self.Ki_v * self.e_v_int
        return float(np.clip(u, -0.4, 0.4))

    def control_angular(self):
        w_ref = self.error_pixeles * self.Kp_vision
        w_ref = np.clip(w_ref, -2.5, 2.5)

        if abs(w_ref) < 0.01:
            self.e_w_int = 0.0

        # Control PI para asegurar que llegamos a esa w_ref
        error_velocidad = w_ref - self.w_real        
        self.e_w_int += error_velocidad * self.dt
        self.e_w_int = np.clip(self.e_w_int, -0.5, 0.5) # Anti-windup
        
        w = (self.Kp_w * error_velocidad) + (self.Ki_w * self.e_w_int)
        return float(np.clip(w, -4.0, 4.0))

    def control_loop(self):
        cmd = Twist()
        
        cmd.linear.x = self.control_linear(self.v_ref)
        cmd.angular.z = self.control_angular()
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()