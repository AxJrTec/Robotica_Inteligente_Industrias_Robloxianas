import rclpy
import numpy as np
import signal
import matplotlib.pyplot as plt
import transforms3d

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class CalcError(Node):
    def __init__(self):
        super().__init__('odometry_robot_node')

        # Set the parameters of the system
        self.rate = 50.0 #Hz

        # Pose del robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Target
        self.x_d = 0.0
        self.y_d = 0.0
        self.th_d = 0.0

        # Historial
        self.x_data = []
        self.y_data = []
        self.xd_data = []
        self.yd_data = []

        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)
        self.sub_target = self.create_subscription(Vector3, 'target', self.target_callback, 10)

        # Publishers
        self.error_pub = self.create_publisher(Vector3, 'error', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.run) # 100 Hz
    
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.line_robot, = self.ax.plot([], [], label='Robot')
        self.line_target, = self.ax.plot([], [], '--', label='Target')
        self.point_robot, = self.ax.plot([], [], 'ro')
        self.point_target, = self.ax.plot([], [], 'go')

        self.ax.set_title("Posición Robot vs Target")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid()

        self.get_logger().info('Gráfica en tiempo real iniciada')

    # Callbacks
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.th = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def target_callback(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y
        self.th_d = msg.z

    def run(self):
        ex = self.x_d - self.x
        ey = self.y_d - self.y
        eth = self.th_d - self.th
        #eth = np.arctan2(np.sin(eth), np.cos(eth))

        msg = Vector3()
        msg.x = ex
        msg.y = ey
        msg.z = eth
        self.error_pub.publish(msg)
        self.get_logger().info(f"E_x: {ex}, E_y: {ey}, E_th: {eth}")

        self.x_data.append(self.x)
        self.y_data.append(self.y)
        self.xd_data.append(self.x_d)
        self.yd_data.append(self.y_d)

        if len(self.x_data) > 200:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.xd_data.pop(0)
            self.yd_data.pop(0)

        self.line_robot.set_data(self.x_data, self.y_data)
        self.line_target.set_data(self.xd_data, self.yd_data)

        self.point_robot.set_data([self.x], [self.y])
        self.point_target.set_data([self.x_d], [self.y_d])

        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.001)

    def stop_handler(self, signum, frame):
        """Handles Ctrl+C (SIGINT)."""
        self.get_logger().info("Deteniendo nodo...")
        plt.close('all')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = CalcError()

    signal.signal(signal.SIGINT, node.stop_handler)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
