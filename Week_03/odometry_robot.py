import rclpy
import numpy as np
import signal
import transforms3d

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class OdometryRobot(Node):
    def __init__(self):
        super().__init__('odometry_robot_node')
        
        #Set the parameters of the system
        self._l = 0.156
        self._r = 0.05
        self._sample_time = 0.01
        self.rate = 100.0 #Hz

        #Variables to be used
        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        self.Omega = 0.0

        #Messages to be used
        self.alpha = 0.85
        self.wr = Float32()
        self.wl = Float32()
        self.wr_f = 0.0
        self.wl_f = 0.0
        self.odom_msg = Odometry()

        #Pose del robot
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0
        
        # Internal state
        self.first = True
        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0

        # Subscriptions
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Timer to update kinematics at ~100Hz
        self.timer = self.create_timer(1.0 / self.rate, self.run) # 100 Hz
        self.get_logger().info('Velocidad segun los sensores')

    # Callbacks
    def encR_callback(self, msg):
        self.wr = msg
        self.wr_f = (self.alpha * self.wr.data) + (1.0 - self.alpha) * self.wr_f
        #self.wr_f = self.wr.data
        self.get_logger().info(f'Vel Derecha: {self.wr_f}')

    def encL_callback(self, msg):
        self.wl = msg
        self.wl_f = (self.alpha * self.wl.data) + (1.0 - self.alpha) * self.wl_f
        #self.wl_f = self.wr.data
        self.get_logger().info(f'Vel Izquierda: {self.wl_f}')

    def run(self):
        if self.first:
            self.start_time = self.get_clock().now()
            self.last_time = self.start_time
            self.current_time = self.start_time
            self.first = False
            return

        # Get current time and compute dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        
        if dt > self._sample_time:
            #Wheel Tangential Velocities
            self.v_r = self._r * self.wr_f
            self.v_l = self._r * self.wl_f

            #Robot Velocities
            self.V = (0.5) * (self.v_r + self.v_l)
            self.Omega = (1.0 / self._l) * (self.v_r - self.v_l)

            self.Th += self.Omega * dt
            self.Th = np.arctan2(np.sin(self.Th), np.cos(self.Th))
            self.X += self.V * np.cos(self.Th) * dt
            self.Y += self.V * np.sin(self.Th) * dt

            self.last_time = current_time

            self.publish_odometry()
            self.get_logger().info(f'Vel_L:{self.V}, Vel_A:{self.Omega}')
            self.get_logger().info(f'X:{self.X} Y:{self.Y} tetha:{self.Th}')


    def publish_odometry(self):
        """ Publishes odometry message with updated state """
        q1 = transforms3d.euler.euler2quat(0, 0, self.Th)

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = "base_footprint"
        
        # Posición
        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0
        
        # Orientación
        self.odom_msg.pose.pose.orientation.x = q1[1]
        self.odom_msg.pose.pose.orientation.y = q1[2]
        self.odom_msg.pose.pose.orientation.z = q1[3]
        self.odom_msg.pose.pose.orientation.w = q1[0]
    
        # Velocidades
        self.odom_msg.twist.twist.linear.x = self.V
        self.odom_msg.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(self.odom_msg)

    def stop_handler(self, signum, frame):
        """Handles Ctrl+C (SIGINT)."""
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = OdometryRobot()

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