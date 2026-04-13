import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import CustomPose

class ControllerLazoAbierto(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(CustomPose, 'pose', self.callback, 10)
        self.targets = []

        # Estados
        self.IDLE = 0
        self.TURN = 1
        self.MOVE = 2
        self.STOP = 3

        self.state = self.IDLE
        self.current = None
        self.state_start_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop)

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

    # Recibimos los datos
    def callback(self, msg):
        self.targets.append(msg)
        self.get_logger().info("Punto agregado")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        if self.state == self.IDLE:
            if len(self.targets) > 0:
                self.current = self.targets.pop(0)
                self.state = self.TURN
                self.state_start_time = now
                self.get_logger().info(f"Nuevo objetivo v={self.current.req_v:.2f}, w={self.current.req_w:.2f}")
            else:
                self.state = self.STOP
                self.get_logger().info("Sin objetivos, pasando a STOP")

        elif self.state == self.TURN:
            cmd.angular.z = self.control_A(self.current.req_w)
            if elapsed >= self.current.t_turn:
                self.state = self.MOVE
                self.state_start_time = now
                self.get_logger().info("Giro completado")

        elif self.state == self.MOVE:
            cmd.linear.x = self.control_L(self.current.req_v)
            if elapsed >= self.current.t_fwd:
                self.state = self.IDLE
                self.get_logger().info("Punto alcanzado")

        elif self.state == self.STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publicar comando
        self.cmd_vel_pub.publish(cmd)
    
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerLazoAbierto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()