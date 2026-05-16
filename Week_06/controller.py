import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool


# Máquina de estados
TRACKING  = 0   # siguiendo la línea normalmente
SEARCHING = 1   # buscando la línea (perdida)

class Controller(Node):
    def __init__(self):
        super().__init__('lazo_cerrado_node')

        # ── Subscribers & Publisher ──────────────────────────────────────────
        self.odom_sub     = self.create_subscription(Odometry, 'odom',           self.odom_callback,     10)
        self.error_sub    = self.create_subscription(Float32,  'error',           self.error_callback,    10)
        self.semaforo_sub = self.create_subscription(String,   'estado_semaforo', self.semaforo_callback, 10)
        self.line_sub     = self.create_subscription(Bool,     'line_detected',   self.line_cb,           10)
        self.cmd_pub      = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── Variables de odometría ───────────────────────────────────────────
        self.v_real = 0.0
        self.w_real = 0.0

        # ── Seguimiento de línea ─────────────────────────────────────────────
        self.v_ref         = 0.1     # m/s velocidad base
        self.Kp_vision     = 0.015
        self.error_pixeles = 0.0
        self.last_error_time = self.get_clock().now()
        self.ERROR_TIMEOUT   = 1.0   # segundos sin error → centrar
        self.line_detected = False

        # ── Semáforo ─────────────────────────────────────────────────────────
        self.ultimo_estado = 'NINGUNO'

        # ── Parámetros PI ────────────────────────────────────────────────────
        self.dt = 0.02
        self.Kp_v, self.Ki_v = 1.1,  0.35
        self.Kp_w, self.Ki_w = 1.3,  0.35
        self.e_v_int = 0.0
        self.e_w_int = 0.0

        # ── Búsqueda de línea ────────────────────────────────────────────────
        self.TRACKING  = TRACKING
        self.SEARCHING = SEARCHING

        self.state         = TRACKING
        self.search_dir    =  1       # 1 = izquierda, -1 = derecha
        self.search_timer  =  0.0     # segundos acumulados buscando
        self.search_period =  1.0     # segundos antes de invertir giro
        self.w_search      =  0.8     # rad/s durante búsqueda
        self.v_search      =  0.03    # m/s durante búsqueda (avance lento)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Controlador iniciado')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.v_real = msg.twist.twist.linear.x
        self.w_real = msg.twist.twist.angular.z

    def error_callback(self, msg):
        self.error_pixeles   = msg.data
        self.last_error_time = self.get_clock().now()

    def line_cb(self, msg):
        self.line_detected = msg.data

    def semaforo_callback(self, msg):
        estado = msg.data
        if estado in ('ROJO', 'AMARILLO', 'VERDE'):
            if estado != self.ultimo_estado:
                self.get_logger().info(f'Semáforo: {self.ultimo_estado} → {estado}')
                self.ultimo_estado = estado

    # ── Control ──────────────────────────────────────────────────────────────
    def control_linear(self, v_ref):
        if v_ref == 0.0:
            self.e_v_int = 0.0
            return 0.0
        e = v_ref - self.v_real
        self.e_v_int += e * self.dt
        self.e_v_int  = np.clip(self.e_v_int, -0.5, 0.5)
        u = self.Kp_v * e + self.Ki_v * self.e_v_int
        return float(np.clip(u, -0.3, 0.3))

    def control_angular(self):
        w_ref = self.error_pixeles * self.Kp_vision
        w_ref = np.clip(w_ref, -2.5, 2.5)
        if abs(w_ref) < 0.01:
            self.e_w_int = 0.0
        error_velocidad  = w_ref - self.w_real
        self.e_w_int    += error_velocidad * self.dt
        self.e_w_int     = np.clip(self.e_w_int, -0.5, 0.5)
        w = self.Kp_w * error_velocidad + self.Ki_w * self.e_w_int
        return float(np.clip(w, -3.0, 3.0))

    # ── Loop principal ────────────────────────────────────────────────────────
    def control_loop(self):
        cmd = Twist()
        # ── Transición de estados ────────────────────────────────────────────
        if self.line_detected:
            if self.state == SEARCHING:
                self.get_logger().info('Línea recuperada → TRACKING')
            self.state        = TRACKING
            self.search_timer = 0.0       # reiniciar timer al recuperar
        else:
            if self.state == TRACKING:
                self.get_logger().info('Línea perdida → SEARCHING')
            self.state = SEARCHING

        # ── Velocidad según semáforo ─────────────────────────────────────────
        if self.ultimo_estado == 'ROJO':
            v_ref = 0.0
        elif self.ultimo_estado == 'AMARILLO':
            v_ref = self.v_ref * 0.5
        else:
            v_ref = self.v_ref

        # ── Acción según estado ──────────────────────────────────────────────
        if self.state == SEARCHING:
            self.search_timer += self.dt
            if self.search_timer >= self.search_period:
                self.search_timer = 0.0
                self.search_dir  *= -1
                self.get_logger().info(
                    f'Búsqueda: cambiando dirección → {"izquierda" if self.search_dir > 0 else "derecha"}'
                )

            # Si el semáforo dice ROJO, no moverse aunque busquemos
            if v_ref == 0.0:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x  = self.v_search
                cmd.angular.z = self.search_dir * self.w_search

        else:  # TRACKING
            # Timeout de error: si la cámara tardó mucho, centrar
            elapsed = (self.get_clock().now() - self.last_error_time).nanoseconds * 1e-9
            if elapsed > self.ERROR_TIMEOUT:
                self.error_pixeles = 0.0

            cmd.linear.x  = self.control_linear(v_ref)
            cmd.angular.z = self.control_angular()

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'[{("TRACKING", "SEARCHING")[self.state]}] '
            f'[Sem: {self.ultimo_estado}] '
            f'v={cmd.linear.x:.3f} | w={cmd.angular.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
